from typing import Type, TypedDict, cast, Optional, Dict, Union, Callable, List
import click
import os
import time
from pathlib import Path
from enum import Enum
import subprocess
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler, FileSystemEvent
import signal


class FileSystemEventHandlerDebounced(FileSystemEventHandler):
    def __init__(
        self,
        callback: Callable[[], Union[None, subprocess.Popen]],
        file_types: List[str] = [],
        debounce_time=2.0,
    ):
        self.callback = callback
        self.file_types = file_types
        self.debounce_time = debounce_time
        self.last_event = ""
        self.last_event_time = 0
        self.process = None
        self.is_terminating = False

    def kill_process(self):
        # Kill existing process
        if type(self.process) is subprocess.Popen:
            self.is_terminating = True
            click.echo(click.style("Sending SIGINT to process...", fg="red"))
            self.process.send_signal(signal.SIGINT)

            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                click.echo(click.style("Sending SIGTERM to process...", fg="red"))
                self.process.send_signal(signal.SIGTERM)
                self.process.wait()

            self.process = None
            click.echo(click.style("Process has exited", fg="yellow"))
            self.is_terminating = False

    def start_process(self):
        process = self.callback()

        if type(process) is subprocess.Popen:
            self.process = process

    def on_modified(self, event: FileSystemEvent):
        # Ignore directories
        if event.is_directory:
            return

        file_type = Path(event.src_path).suffix

        # Ignore files that don't match the file types
        if file_type not in self.file_types:
            return

        # Debounce
        if (
            event.src_path == self.last_event
            and time.time() - self.last_event_time < self.debounce_time
        ):
            return

        # Is the process currently being terminated
        if self.is_terminating:
            return

        self.last_event = event.src_path
        self.last_event_time = time.time()

        self.kill_process()
        self.start_process()


class RosEnv(TypedDict):
    PLATFORM_MODULE: str
    ROS_OVERLAY: str


class LogLevels(Enum):
    WARNING = "warning"
    ERROR = "error"


def check_directory_ownership(path: Path) -> bool:
    stat = path.stat()
    return stat.st_uid == os.getuid() and stat.st_gid == os.getgid()


def get_env(env_type: Union[Type[TypedDict], Type[TypedDict]], abort: bool = True) -> TypedDict:
    if abort:
        for env in env_type.__required_keys__:  # type: ignore
            if env not in os.environ:
                raise click.ClickException(f"{env} environment variable must be set.")

    return cast(env_type, {k: os.environ[k] for k in env_type.__required_keys__})


def get_ros_env(abort: bool = True) -> RosEnv:
    return get_env(RosEnv, abort)


def is_ci_environment() -> bool:
    return (os.environ.get("CI") or "").lower() == "true"


def echo(
    msg: str = "",
    color: str = "blue",
    group_start: bool = False,
    group_end: bool = False,
    level: Optional[LogLevels] = None,
):
    """Echo a message to the console, if we are in a github actions environment, log it to the github actions log"""
    if is_ci_environment():
        if group_end:
            print("::endgroup::")
        if group_start:
            print(f"::group::{msg}")
        else:
            # See https://docs.github.com/en/actions/using-workflows/workflow-commands-for-github-actions#example-setting-an-error-message
            if level:
                print(f"::{level.value}::{msg}")
            else:
                print(msg)
    else:
        click.echo(click.style(msg, fg=color))  # type: ignore


def get_project_root() -> Optional[Path]:
    # not sure of a great way to find the project root, find the first .git directory?

    p = Path.cwd()

    while len(p.parts) > 1:
        if (p / ".git").exists() and (p / "package.json").exists():
            return p
        else:
            p = p.parent

    return None


def stdout_call(
    command: str,
    cwd: Optional[Path] = None,
    project_root_cwd: bool = False,
    abort: bool = True,
) -> str:
    if project_root_cwd and cwd:
        raise RuntimeError("Both 'cwd' and 'project_root_cwd' are set")

    if project_root_cwd:
        cwd = get_project_root()
        if cwd is None:
            raise RuntimeError("Could not find project root.")

    click.echo(
        click.style(
            f"Running: {click.style(command, bold=True)} in {click.style(str(cwd if cwd else Path.cwd()), bold=True)}",
            fg="blue",
        )
    )
    try:
        proc = subprocess.run(
            command,
            shell=True,
            executable="/bin/bash",
            capture_output=True,
            cwd=cwd,
            check=abort,
        )
    except subprocess.CalledProcessError as e:
        print(e)
        raise click.ClickException("Run failed")

    return proc.stdout.decode("ascii")


def start_watcher(
    callback,
    file_types: List[str] = [".py", ".cpp", ".c", ".hpp", ".h"],
    debounce_time: float = 2.0,
):
    folders_to_ignore = ["log", "build"]

    # Get all folders in the current directory
    cwd = Path().cwd()
    folders = [f for f in os.listdir(cwd) if os.path.isdir(f) and f not in folders_to_ignore]

    click.echo(
        click.style(
            f"Watching: {click.style(cwd, bold=True)}",
            fg="blue",
        )
    )
    handler = FileSystemEventHandlerDebounced(callback, file_types, debounce_time)
    observer = Observer()

    for folder in folders:
        # Watch all folders in the current directory
        observer.schedule(handler, cwd / folder, recursive=True)

    observer.start()

    # Trigger a fake event to start the process
    handler.on_modified(FileSystemEvent("fake_event.py"))

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        handler.kill_process()
    finally:
        observer.stop()
        observer.join()


def call(
    command: str,
    cwd: Optional[Path] = None,
    project_root_cwd: bool = False,
    abort: bool = True,
    sudo: bool = False,
    env: Dict[str, str] = {},
    retry: int = 0,
    process: bool = False,  # return the process object
) -> Union[subprocess.CompletedProcess[bytes], subprocess.Popen[bytes]]:
    if project_root_cwd and cwd:
        raise RuntimeError("Both 'cwd' and 'project_root_cwd' are set")

    if project_root_cwd:
        cwd = get_project_root()
        if cwd is None:
            raise RuntimeError("Could not find project root.")

    env_extended = {**os.environ, **env}

    if sudo:
        command = "sudo " + command

    click.echo(
        click.style(
            f"Running: {click.style(command, bold=True)} in {click.style(str(cwd if cwd else Path.cwd()), bold=True)}",
            fg="blue",
        )
    )

    proc = None
    if process:
        proc = subprocess.Popen(
            command, shell=True, executable="/bin/bash", cwd=cwd, env=env_extended
        )
        return proc

    try:
        with subprocess.Popen(
            command, shell=True, executable="/bin/bash", cwd=cwd, env=env_extended
        ) as process:
            # this is the internal time to wait for the process to exit after SIGINT, and then SIGTERM is sent
            process._sigint_wait_secs = 10.0
            try:
                stdout, stderr = process.communicate(input, timeout=None)
            except Exception as e:  # Including KeyboardInterrupt, communicate handled that.
                # looking at the python stdlib code, the SIGINT is already sent in the communicate/wait method
                # so if we reach this point the process hasn't exited yet, so we need to send SIGTERM
                print("Sending SIGTERM to process")
                process.send_signal(signal.SIGTERM)
                process.wait()
                raise e
            retcode = process.poll()
            if abort and retcode:
                raise subprocess.CalledProcessError(
                    retcode, process.args, output=stdout, stderr=stderr
                )
        return subprocess.CompletedProcess(process.args, retcode, stdout, stderr)

    except subprocess.CalledProcessError as e:
        if retry > 0:
            click.echo(
                click.style(
                    f"Failed but retrying... {retry - 1} retry(s) left",
                    fg="yellow",
                )
            )
            return call(command, cwd, project_root_cwd, abort, sudo, env, retry - 1)
        else:
            print(e)
            raise click.ClickException("Run failed")
