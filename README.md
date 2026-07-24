# navmesh-tools

Navigation-mesh generation and pathfinding for game AI / robotics path planning — build a queryable navmesh from raw walkable triangle geometry plus agent radius/height/slope/climb, then query it for shortest paths, straightened walkable lines, raycasts, nearest-polygon snapping, deterministic random sampling, constrained surface sliding, walkable-surface height, and spatial polygon lookups. Built for the [Axiom](https://axiomide.com) marketplace, wrapping [recast4j](https://github.com/recast4j/recast4j) (zlib license), the pure-Java port of Mikko Mononen's Recast & Detour navmesh toolkit.

## Use it from your agent or app

Every node in this package is a **live, auto-scaling API endpoint** on the
[Axiom](https://axiomide.com) marketplace — call it from an AI agent or your
own code, with nothing to self-host.

**📦 See it on the marketplace:**
https://dev.axiomide.com/marketplace/christiangeorgelucas/navmesh-tools@0.1.0

**Hook it up to an AI agent (MCP).** Add Axiom's hosted MCP server to any MCP
client and every node becomes a typed tool your agent can call — search the
catalog, inspect a schema, and invoke it directly.

```bash
# Claude Code
claude mcp add --transport http axiom https://api.axiomide.com/mcp \
  --header "Authorization: Bearer $AXIOM_API_KEY"
```

Claude Desktop, Cursor, or any config-based client:

```json
{
  "mcpServers": {
    "axiom": {
      "type": "http",
      "url": "https://api.axiomide.com/mcp",
      "headers": { "Authorization": "Bearer YOUR_AXIOM_API_KEY" }
    }
  }
}
```

**Call it from the CLI.**

```bash
axiom invoke christiangeorgelucas/navmesh-tools/FindPath --input '{"navmesh":{"data":"<base64 navmesh bytes from BuildNavMesh>","agentRadius":0.6,"agentHeight":2.0},"start":{"x":2,"y":0,"z":2},"end":{"x":8,"y":0,"z":8}}'
```

**Call it over HTTP.**

```bash
curl -X POST https://api.axiomide.com/invocations/v1/nodes/christiangeorgelucas/navmesh-tools/0.1.0/FindPath \
  -H "Authorization: Bearer $AXIOM_API_KEY" \
  -H 'Content-Type: application/json' \
  -d '{"navmesh":{"data":"<base64 navmesh bytes from BuildNavMesh>","agentRadius":0.6,"agentHeight":2.0},"start":{"x":2,"y":0,"z":2},"end":{"x":8,"y":0,"z":8}}'
```

### Get started free

Install the CLI:

```bash
# macOS / Linux — Homebrew
brew install axiomide/tap/axiom

# macOS / Linux — install script
curl -fsSL https://raw.githubusercontent.com/AxiomIDE/axiom-releases/main/install.sh | sh
```

**Windows:** download the `windows/amd64` `.zip` from the
[releases page](https://github.com/AxiomIDE/axiom-releases/releases), unzip
it, and put `axiom.exe` on your `PATH`.

Then `axiom version` to verify, `axiom login` (GitHub or Google) to
authenticate, and create an API key under **Console → API Keys**. Docs and
sign-up at **[axiomide.com](https://axiomide.com)**.

## What this is

A navmesh is the standard data structure game AI and mobile-robot path
planners use to answer "can an agent of this size get from A to B, and how":
walkable floor geometry is voxelized, eroded by the agent's radius, and
triangulated into a mesh of convex polygons an agent can legally stand on.
`navmesh-tools` wraps [recast4j](https://github.com/recast4j/recast4j)'s
`recast` (mesh building) and `detour` (mesh querying) modules — the
authoritative Java port of the Recast & Detour C++ libraries used throughout
the game industry (Unity, Unreal, Godot's navmesh systems all trace back to
the same algorithm).

Every node is **stateless and deterministic**: `BuildNavMesh` always produces
a byte-identical navmesh for the same geometry + agent config, and every
query node is a pure function of the (serialized) navmesh and its arguments —
no randomness escapes a caller-supplied `seed`. Geometry is always supplied
**inline** by the caller as flat vertex/triangle arrays — nodes never fetch a
file or URL.

## Nodes

| Node | What it does |
|---|---|
| `BuildNavMesh` | Build a queryable navmesh from raw triangle geometry + an agent config (radius, height, max slope, max climb, voxel cell size/height). |
| `FindPath` | Shortest polygon-reference corridor between two points. |
| `FindStraightPath` | Straighten a `FindPath` corridor into the actual walkable line (minimal waypoint corners). |
| `Raycast` | Cast a ray across the walkable surface; report the first wall/edge hit, or full reachability. |
| `FindNearestPoly` | Snap an arbitrary point to the nearest navmesh polygon within a search box. |
| `FindRandomPoint` | Sample a uniformly-random point anywhere on the walkable surface, seeded for determinism. |
| `FindRandomPointAroundCircle` | Sample a uniformly-random reachable point within a radius of a center, seeded for determinism. |
| `MoveAlongSurface` | Slide from one point toward another, constrained to the walkable surface (clipped at walls). |
| `GetPolyHeight` | Look up the walkable-surface height under a given (x, z). |
| `QueryPolygonsInBox` | List every polygon whose bounds intersect an axis-aligned box. |
| `BuildAndFindPath` | One-shot convenience: build + find + straighten a path in a single call. |

**Out of scope** (see the retrospective for the full enumeration/omission
log): crowd/multi-agent simulation, dynamic obstacle rebuilding, and
off-mesh connections are stateful or advanced authoring concerns better
suited to a game engine's own runtime than a stateless pure-function node.

## Distinct from other Axiom packages

- **`graph-tools`** — abstract graph shortest-path over a caller-supplied
  adjacency structure. No geometry involved.
- **`mesh-tools`** — 3D mesh measurement/repair/booleans (volume, surface
  area, decimation). Not navigation.
- **`geometry-tools` / `spatial-tools`** — 2D planar / N-dimensional
  point-set computational geometry (JTS / SciPy). No navmesh, no agent
  clearance, no pathfinding over a built mesh.

`navmesh-tools`'s distinctive capability: **from raw walkable geometry, build
a navmesh, then query it for agent movement.**

## License

MIT (this package). Wraps [recast4j](https://github.com/recast4j/recast4j)
(`org.recast4j:recast`, `org.recast4j:detour`), licensed under the
[zlib License](https://github.com/recast4j/recast4j/blob/master/License.txt) —
verified from the project's own `License.txt` and Maven Central POM metadata.
Neither module used here pulls in any further runtime dependency.
