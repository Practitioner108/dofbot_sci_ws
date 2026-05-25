# Deprecated URDF files

Use `../dofbot.xacro` for all current work.

- **dofbot.urdf** — Old exported URDF with joint1-joint5 naming convention. Visual meshes originally referenced uppercase `.STL` which fails on case-sensitive Linux filesystems (now fixed, but the naming convention is incompatible with all runtime code).
- **dofbot_orgin.urdf** — Yahboom official original URDF. Uses simplified collision geometry, different joint/link names, and flat mesh path layout. Preserved for historical reference only.
