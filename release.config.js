module.exports = {
    branches: ["main"],
    plugins: [
        ["@semantic-release/commit-analyzer", { "preset": "conventionalcommits" }],
        ["@semantic-release/release-notes-generator", { "preset": "conventionalcommits" }],
        "@semantic-release/changelog",
        [
            "@semantic-release/exec",
            {
              "prepareCmd": "echo __version__ = \"${nextRelease.version}\" > ./platform_cli/__init__.py",
            }
        ],
        "@semantic-release/github",
        [
            "@semantic-release/git",
            {
              "assets": [
                "CHANGELOG.md",
                "./platform_cli/__init__.py"
              ]
            }
          ]
    ],
};