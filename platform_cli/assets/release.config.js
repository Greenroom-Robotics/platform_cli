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
                "prepareCmd": "platform pkg build --version ${nextRelease.version}",
                "publishCmd": "platform pkg apt-clone && platform pkg apt-add && platform pkg apt-push"
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