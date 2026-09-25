# DTrackSDK upstream provenance

- Project: DTrackSDK
- Vendor: Advanced Realtime Tracking GmbH & Co. KG (ART)
- Repository: https://github.com/ar-tracking/DTrackSDK
- Bundled version: 2.9.0
- Upstream tag: `v2.9.0`
- Upstream commit: `110f7e5edf8b93d62d22cd22eba41d5197b7b932`
- License: BSD 3-Clause; see `license.txt`

The SDK headers and implementation files in this directory are vendored so
that `dtrack_publisher` can be built without a separately installed DTrackSDK
or network access. They are compiled as a private static library and are not
part of the public ROS package API.

When updating the SDK, use a specific upstream release tag rather than the
moving `master` branch. Replace the vendored SDK files with the unmodified
files from that tag, update `VERSION` and this provenance record, adapt the
private source list in the package's `CMakeLists.txt` if necessary, review the
upstream license, and run the full package build and test suite.
