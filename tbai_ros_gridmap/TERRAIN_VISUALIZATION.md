# Terrain visualization across worlds

Shared convex decomposition configurations use the original
`preprocessing.resolution: 0.04` resampling resolution. Median filtering and
inpainting remain enabled.

The application RViz displays subscribed to
`/convex_plane_decomposition_ros/filtered_map` use `elevation_before_postprocess`
for height and, where applicable, color. This shows the preprocessed surface
before controller clearance inflation. Controller postprocessing offsets remain
unchanged.

These displays enable **Cell Walls**: every valid cell has a flat square top,
and unequal adjacent heights are joined by a vertical face at their shared edge.
Unknown cells and the outer map boundary remain open; no base height is invented.
Grid Cell Decimation selects cell outlines without decimating the filled surface.
The renderer uses up to 12 vertices per cell instead of one.

Restart RViz to load the rebuilt plugin and updated configuration. Restart the
mapping/decomposition pipeline to load the updated preprocessing parameters.
For custom RViz configurations, enable Cell Walls and select
`elevation_before_postprocess` as the height/color layer manually.

Cell Walls works in every world but renders ramps and noisy surfaces as small
steps too. Turn it off to use the original interpolated triangle surface for
smooth terrain. Its plugin default remains off for compatibility with custom
configurations. Boundaries follow the input grid, not exact simulated geometry.
