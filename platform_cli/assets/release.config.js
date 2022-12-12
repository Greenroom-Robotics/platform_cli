module.exports = {
    "branches": [
        "main",
        {
            "name": "alpha",
            "prerelease": true
        }
    ],
    "plugins": [
        [
            "@semantic-release/commit-analyzer",
            {
                "preset": "conventionalcommits"
            }
        ],
        [
            "@semantic-release/release-notes-generator",
            {
                "preset": "conventionalcommits"
            }
        ],
        "@semantic-release/changelog",
        [
            "@semantic-release/exec",
            {
                "prepareCmd": "platform release deb-prepare --version ${nextRelease.version}",
                "publishCmd": "platform release deb-publish"
            }
        ],
        [
            "@semantic-release/github",
            {
                "assets": [
                    {
                        "path": "**/*.deb"
                    }
                ],
                "successComment": false
            }
        ],
        [
            "@semantic-release/git",
            {
                "assets": [
                    "CHANGELOG.md"
                ]
            }
        ]
    ]
};