module.exports = {
    branches: ["main"],
    plugins: [
        ["@semantic-release/commit-analyzer", { "preset": "conventionalcommits" }],
        ["@semantic-release/release-notes-generator", { "preset": "conventionalcommits" }],
        "@semantic-release/changelog",
        [
            "@semantic-release/exec",
            {
              "prepareCmd": "./scripts/version.sh ${nextRelease.version}",
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