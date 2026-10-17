# Verification map

Start from the repository root with the isolated `$run` directory created by
the [skill](../SKILL.md). Scripts create unique child directories. Docker runs
use isolated networks and do not share ROS processes or output directories.
Do not double-drive one live Fusion design.

- [Fusion export](fusion-export.md) verifies the actual CAD extraction and STL boundary.
- [Cross-tool loading](cross-tool-loading.md) verifies mesh loading, scale, placement, and FK.
- [ROS 2 runtime](ros2-runtime.md) verifies install-space launches and commanded articulation.

Report each entry point separately. If Fusion or a runtime is absent, mark
that feature unverified rather than substituting a fixture or another tool.
