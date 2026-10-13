# Releases

All packages in this repository, including tests and tutorials, share one version.
The release-please manifest starts at the existing `2.0.3` tag.
The config pins `bootstrap-sha` to that tag's commit because it has no corresponding GitHub Release.
This limits the first release PR to changes after that tag; release-please ignores the boundary after its first release.
Aligning package metadata with that baseline does not create a release or change existing tags.

## Tag releases

1. Squash merge PRs with conventional titles such as `fix: ...` or `feat: ...`.
   Use `feat!: ...` or a `BREAKING CHANGE:` footer for breaking changes.
   The existing PR-title check enforces the conventional format.
2. Release-please opens or updates a release PR after pushes to `main`.
   It updates every `package.xml`, Python `setup.py` version, the manifest, and the root `CHANGELOG.md`.
   Features bump the minor version, fixes bump the patch, and breaking changes bump the major.
3. Review the version, changes, and CI results, then merge the release PR.
   Release-please creates an unprefixed tag such as `2.1.0` and a GitHub Release on the next push run.
   This workflow publishes source tags and release notes only.

Enable **Allow GitHub Actions to create and approve pull requests** in repository Actions settings.
The workflow needs Contents, Issues, and Pull requests write permissions.
It does not automatically merge release PRs.

For automatic CI on generated release PRs, add the repository Actions secret `RELEASE_PLEASE_TOKEN`.
Use a PAT owned by someone with repository write access.
A fine-grained PAT needs Contents, Issues, and Pull requests write permissions for this repository.
A classic PAT needs `public_repo` for this public repository, with organization SSO authorization where required.
Without this secret, the workflow uses `GITHUB_TOKEN`, whose generated PRs and tags do not trigger other workflows.
In that mode, a maintainer must arrange CI on the release PR before merging it, for example by closing and reopening it manually.
A manual dispatch of Release Please only validates versions; it does not create releases or run the ROS test matrix.

Validate locally from the repository root:

```bash
python3 -m unittest discover -s tools/tests -q
python3 tools/check_release_versions.py .
```

When adding a package, include its `package.xml` and any `setup.py` in `release-please-config.json` under `extra-files`.
Keep the `x-release-please-version` comment on the Python version line.
XML version elements are updated directly by release-please.
CI rejects package version drift and missing update coverage.

## Future ROS distribution releases

The fully commented Bloom job in `.github/workflows/release-please.yml` is an **untested example** for a maintainer to review.
It cannot run in its current form.
Jazzy already uses the release repository `ros2-gbp/nodl-release`.
Validate its track and package selection before enabling any automation.
New distribution tracks require separate setup.

The example expects Actions secret `BLOOM_RELEASE_PAT` and repository variable `BLOOM_GITHUB_USER`.
Use a classic PAT with `public_repo` and `workflow` scopes.
Its owner needs push access to `ros2-gbp/nodl-release`, permission to create/update their own `ros/rosdistro` fork, and permission to open a PR against `ros/rosdistro`.
Authorize organization SSO where required.
The source repository's `GITHUB_TOKEN` cannot replace these cross-repository credentials.
Bloom reads API credentials from `~/.config/bloom`; Git push authentication is configured separately in the example.

Before enabling Bloom:

- Confirm the track imports the new unprefixed tag using `release-tag: :{version}`.
  Check that automatic version selection cannot pick a newer version from `main`.
- Review included and ignored packages, especially the new generators, test packages, and tutorials.
- Maintain package `CHANGELOG.rst` files for ROS releases; release-please updates the root Markdown changelog only.
- Check rosdep dependencies and packaging for the target distribution.
- Test and pin the Bloom toolchain, then validate one distribution interactively with the release maintainer.

Release-please owns upstream version bumps and tags.
Do not also run `catkin_prepare_release`, which would bump and tag again.
Bloom generates distribution packaging and a rosdistro PR from an existing upstream release.

References:

- [Release-please configuration](https://github.com/googleapis/release-please/blob/main/docs/customizing.md)
- [CI on release-please PRs](https://github.com/googleapis/release-please-action#other-actions-on-release-please-prs)
- [ROS release tracks](https://docs.ros.org/en/rolling/How-To-Guides/Releasing/Release-Track.html)
- [Bloom credentials and PAT scopes](https://github.com/ros-infrastructure/bloom/blob/master/bloom/github.py)
