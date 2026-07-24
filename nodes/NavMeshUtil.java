package nodes;

import gen.Messages.AgentConfig;
import gen.Messages.Geometry;
import gen.Messages.Vec3;

import org.recast4j.detour.DefaultQueryFilter;
import org.recast4j.detour.MeshData;
import org.recast4j.detour.NavMeshBuilder;
import org.recast4j.detour.NavMeshDataCreateParams;
import org.recast4j.detour.NavMeshQuery;
import org.recast4j.detour.io.MeshSetReader;
import org.recast4j.detour.io.MeshSetWriter;
import org.recast4j.recast.AreaModification;
import org.recast4j.recast.RecastBuilder;
import org.recast4j.recast.RecastBuilder.RecastBuilderResult;
import org.recast4j.recast.RecastBuilderConfig;
import org.recast4j.recast.RecastConfig;
import org.recast4j.recast.RecastConstants.PartitionType;
import org.recast4j.recast.PolyMesh;
import org.recast4j.recast.PolyMeshDetail;
import org.recast4j.recast.geom.SingleTrimeshInputGeomProvider;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.ByteOrder;

/**
 * Shared boilerplate for every node in this package: validate untrusted
 * {@link Geometry}/{@link AgentConfig} input, build a recast4j navmesh from
 * it, serialize/deserialize the navmesh so it can travel between nodes as
 * plain bytes, convert {@link Vec3} <-> float[3], and build structured
 * errors instead of letting a recast4j exception escape a node raw.
 */
final class NavMeshUtil {
    private NavMeshUtil() {
    }

    // A single walkable-ground area id (recast4j's own SampleAreaModifications
    // class is test-only in the published jars, so we define our own here).
    private static final AreaModification WALKABLE_AREA_MOD = new AreaModification(1);

    /** Thrown internally to unwind straight to a structured error. */
    static final class NavOpException extends RuntimeException {
        final String code;

        NavOpException(String code, String message) {
            super(message);
            this.code = code;
        }
    }

    static NavOpException err(String code, String message) {
        return new NavOpException(code, message);
    }

    static String toError(NavOpException e) {
        return e.code + ": " + e.getMessage();
    }

    // ---------------------------------------------------------------
    // Vec3 <-> float[3]
    // ---------------------------------------------------------------

    static float[] toArr(Vec3 v) {
        if (v == null) {
            return new float[] { 0f, 0f, 0f };
        }
        return new float[] { v.getX(), v.getY(), v.getZ() };
    }

    static Vec3 fromArr(float[] a) {
        return Vec3.newBuilder().setX(a[0]).setY(a[1]).setZ(a[2]).build();
    }

    static boolean isFinite3(float[] a) {
        return a != null && a.length == 3 && Float.isFinite(a[0]) && Float.isFinite(a[1]) && Float.isFinite(a[2]);
    }

    // ---------------------------------------------------------------
    // Geometry / AgentConfig validation
    // ---------------------------------------------------------------

    static float[] validateVertices(Geometry g) {
        if (g == null || g.getVerticesCount() == 0) {
            throw err("EMPTY_GEOMETRY", "geometry.vertices is empty");
        }
        if (g.getVerticesCount() % 3 != 0) {
            throw err("INVALID_GEOMETRY", "vertices length must be a multiple of 3, got " + g.getVerticesCount());
        }
        float[] verts = new float[g.getVerticesCount()];
        for (int i = 0; i < verts.length; i++) {
            float f = g.getVertices(i);
            if (!Float.isFinite(f)) {
                throw err("INVALID_GEOMETRY", "vertices contains a non-finite value at index " + i);
            }
            verts[i] = f;
        }
        return verts;
    }

    static int[] validateTriangles(Geometry g, int vertCount) {
        if (g == null || g.getTrianglesCount() == 0) {
            throw err("EMPTY_GEOMETRY", "geometry.triangles is empty");
        }
        if (g.getTrianglesCount() % 3 != 0) {
            throw err("INVALID_GEOMETRY", "triangles length must be a multiple of 3, got " + g.getTrianglesCount());
        }
        int[] tris = new int[g.getTrianglesCount()];
        for (int i = 0; i < tris.length; i++) {
            int idx = g.getTriangles(i);
            if (idx < 0 || idx >= vertCount) {
                throw err("INVALID_GEOMETRY",
                        "triangle index " + idx + " at position " + i + " is out of range [0," + vertCount + ")");
            }
            tris[i] = idx;
        }
        return tris;
    }

    static void validateAgent(AgentConfig a) {
        if (a == null) {
            throw err("INVALID_AGENT_CONFIG", "agent config not set");
        }
        if (!(a.getRadius() > 0f) || !Float.isFinite(a.getRadius())) {
            throw err("INVALID_AGENT_CONFIG", "agent.radius must be > 0, got " + a.getRadius());
        }
        if (!(a.getHeight() > 0f) || !Float.isFinite(a.getHeight())) {
            throw err("INVALID_AGENT_CONFIG", "agent.height must be > 0, got " + a.getHeight());
        }
        if (!(a.getCellSize() > 0f) || !Float.isFinite(a.getCellSize())) {
            throw err("INVALID_AGENT_CONFIG", "agent.cell_size must be > 0, got " + a.getCellSize());
        }
        if (!(a.getCellHeight() > 0f) || !Float.isFinite(a.getCellHeight())) {
            throw err("INVALID_AGENT_CONFIG", "agent.cell_height must be > 0, got " + a.getCellHeight());
        }
        if (a.getMaxSlopeDeg() < 0f || a.getMaxSlopeDeg() >= 90f || !Float.isFinite(a.getMaxSlopeDeg())) {
            throw err("INVALID_AGENT_CONFIG", "agent.max_slope_deg must be in [0,90), got " + a.getMaxSlopeDeg());
        }
        if (a.getMaxClimb() < 0f || !Float.isFinite(a.getMaxClimb())) {
            throw err("INVALID_AGENT_CONFIG", "agent.max_climb must be >= 0, got " + a.getMaxClimb());
        }
    }

    // ---------------------------------------------------------------
    // Build
    // ---------------------------------------------------------------

    static final class Built {
        final org.recast4j.detour.NavMesh navMesh;
        final int polyCount;
        final int vertCount;
        final float[] bmin;
        final float[] bmax;

        Built(org.recast4j.detour.NavMesh navMesh, int polyCount, int vertCount, float[] bmin, float[] bmax) {
            this.navMesh = navMesh;
            this.polyCount = polyCount;
            this.vertCount = vertCount;
            this.bmin = bmin;
            this.bmax = bmax;
        }
    }

    /** Build a single-tile recast4j navmesh from validated geometry + agent config. */
    static Built buildNavMesh(Geometry geometry, AgentConfig agent) {
        float[] verts = validateVertices(geometry);
        int[] tris = validateTriangles(geometry, verts.length / 3);
        validateAgent(agent);

        int vertsPerPoly = 6;
        SingleTrimeshInputGeomProvider geom = new SingleTrimeshInputGeomProvider(verts, tris);

        RecastConfig cfg;
        RecastBuilderResult rcResult;
        try {
            cfg = new RecastConfig(PartitionType.WATERSHED, agent.getCellSize(), agent.getCellHeight(), agent.getHeight(),
                    agent.getRadius(), agent.getMaxClimb(), agent.getMaxSlopeDeg(), 8, 20, 12.0f, 1.3f, vertsPerPoly, 6.0f,
                    1.0f, WALKABLE_AREA_MOD);
            RecastBuilderConfig bcfg = new RecastBuilderConfig(cfg, geom.getMeshBoundsMin(), geom.getMeshBoundsMax());
            rcResult = new RecastBuilder().build(geom, bcfg);
        } catch (NavOpException e) {
            throw e;
        } catch (Exception e) {
            throw err("BUILD_FAILED", "recast rasterization/region build failed: " + e.getMessage());
        }

        PolyMesh pmesh = rcResult.getMesh();
        PolyMeshDetail dmesh = rcResult.getMeshDetail();
        if (pmesh == null || pmesh.npolys == 0 || pmesh.nverts == 0) {
            throw err("NO_WALKABLE_SURFACE",
                    "no walkable polygon could be built from this geometry with the given agent config");
        }

        try {
            for (int i = 0; i < pmesh.npolys; ++i) {
                pmesh.flags[i] = 1;
            }
            NavMeshDataCreateParams params = new NavMeshDataCreateParams();
            params.verts = pmesh.verts;
            params.vertCount = pmesh.nverts;
            params.polys = pmesh.polys;
            params.polyAreas = pmesh.areas;
            params.polyFlags = pmesh.flags;
            params.polyCount = pmesh.npolys;
            params.nvp = pmesh.nvp;
            if (dmesh != null) {
                params.detailMeshes = dmesh.meshes;
                params.detailVerts = dmesh.verts;
                params.detailVertsCount = dmesh.nverts;
                params.detailTris = dmesh.tris;
                params.detailTriCount = dmesh.ntris;
            }
            params.walkableHeight = agent.getHeight();
            params.walkableRadius = agent.getRadius();
            params.walkableClimb = agent.getMaxClimb();
            params.bmin = pmesh.bmin;
            params.bmax = pmesh.bmax;
            params.cs = pmesh.cs;
            params.ch = pmesh.ch;
            params.buildBvTree = true;

            MeshData meshData = NavMeshBuilder.createNavMeshData(params);
            if (meshData == null) {
                throw err("BUILD_FAILED", "navmesh data assembly produced no data");
            }
            org.recast4j.detour.NavMesh navMesh = new org.recast4j.detour.NavMesh(meshData, vertsPerPoly, 0);
            return new Built(navMesh, pmesh.npolys, pmesh.nverts, pmesh.bmin.clone(), pmesh.bmax.clone());
        } catch (NavOpException e) {
            throw e;
        } catch (Exception e) {
            throw err("BUILD_FAILED", "navmesh assembly failed: " + e.getMessage());
        }
    }

    // ---------------------------------------------------------------
    // Serialize / deserialize
    // ---------------------------------------------------------------

    static byte[] serialize(org.recast4j.detour.NavMesh navMesh) {
        try {
            ByteArrayOutputStream out = new ByteArrayOutputStream();
            new MeshSetWriter().write(out, navMesh, ByteOrder.LITTLE_ENDIAN, false);
            return out.toByteArray();
        } catch (IOException e) {
            throw err("BUILD_FAILED", "failed to serialize navmesh: " + e.getMessage());
        }
    }

    static org.recast4j.detour.NavMesh deserialize(byte[] data) {
        if (data == null || data.length == 0) {
            throw err("INVALID_NAVMESH", "navmesh.data is empty");
        }
        try {
            return new MeshSetReader().read(new ByteArrayInputStream(data));
        } catch (Exception e) {
            throw err("INVALID_NAVMESH", "navmesh.data could not be parsed: " + e.getMessage());
        }
    }

    static NavMeshQuery queryFor(gen.Messages.NavMesh navMeshMsg) {
        if (navMeshMsg == null) {
            throw err("INVALID_NAVMESH", "navmesh not set");
        }
        org.recast4j.detour.NavMesh navMesh = deserialize(navMeshMsg.getData().toByteArray());
        try {
            return new NavMeshQuery(navMesh);
        } catch (Exception e) {
            throw err("INVALID_NAVMESH", "navmesh could not be loaded for querying: " + e.getMessage());
        }
    }

    static DefaultQueryFilter defaultFilter() {
        return new DefaultQueryFilter();
    }

    /** A sane default search box when the caller supplies an all-zero half-extents. */
    static float[] defaultHalfExtents(gen.Messages.NavMesh navMeshMsg, Vec3 supplied) {
        float[] s = toArr(supplied);
        if (s[0] != 0f || s[1] != 0f || s[2] != 0f) {
            return s;
        }
        float radius = navMeshMsg.getAgentRadius();
        float height = navMeshMsg.getAgentHeight();
        float hx = Math.max(radius * 4f, 0.5f);
        float hy = Math.max(height * 2f, 0.5f);
        return new float[] { hx, hy, hx };
    }
}
