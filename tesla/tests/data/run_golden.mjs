import fs from 'fs';
import {analyzeRoute} from './engine.mjs';
const [,, inp, out] = process.argv;
const pts = JSON.parse(fs.readFileSync(inp)).map(p => ({lat:p[0], lon:p[1]}));
const r = analyzeRoute(pts);
const slim = r.curves.map(c => ({distanceFromStart:c.distanceFromStart, severity:c.severity, direction:c.direction, minRadius:c.minRadius, length:c.length, advisorySpeedMs:c.advisorySpeedMs ?? null, advisorySpeedKmh:c.advisorySpeedKmh ?? null, startIndex:c.startIndex, endIndex:c.endIndex, modifiers:[...c.modifiers].sort(), compoundType:c.compoundType ?? null, compound:c.compoundType ?? null}));
fs.writeFileSync(out, JSON.stringify({stats:r.stats, curves:slim}, null, 1));
console.log(inp, JSON.stringify(r.stats));
