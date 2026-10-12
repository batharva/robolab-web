#!/usr/bin/env python3
"""Assemble the single-file index.html from src/ (development helper; not needed at runtime)."""
import pathlib
r = pathlib.Path(__file__).parent; s = r / 'src'
rd = lambda n: (s / n).read_text(encoding='utf-8')
app = ''.join(rd(n) for n in ['app_a.js', 'app_b.js', 'app_c.js', 'app_d.js'])
html = f'''<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>RoboLab Web — Interactive Robotics Analysis Studio</title>
<meta name="description" content="Browser-based robotics analysis: DH parameters, forward/inverse kinematics, Jacobian, trajectories, dynamics and workspace analysis.">
<style>
{rd('style.css')}
</style>
</head>
<body>
{rd('body.html')}
<!-- External dependency (only one): Three.js r128 from the jsDelivr CDN. If it fails to load, the maths panels and graphs still work. -->
<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/build/three.min.js" crossorigin="anonymous"></script>
<script>
{rd('core.js')}
</script>
<script>
{app}
</script>
</body>
</html>
'''
(r / 'index.html').write_text(html, encoding='utf-8')
print('index.html', len(html) // 1024, 'KB')
