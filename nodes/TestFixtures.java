package nodes;

import axiom.AxiomContext;
import gen.Messages.AgentConfig;
import gen.Messages.BuildNavMeshInput;
import gen.Messages.BuildNavMeshOutput;
import gen.Messages.Geometry;
import gen.Messages.NavMesh;
import gen.Messages.Vec3;

/**
 * Shared test geometry: a single flat 10x10 quad (two triangles) lying on
 * the XZ ground plane at y=0, Y-up. Winding is CCW-from-above so the
 * triangles face +Y (walkable) — see the retrospective for the hand-worked
 * cross-product derivation.
 *
 * Corners: (0,0,0) (10,0,0) (10,0,10) (0,0,10). After eroding by the
 * default test agent's 0.6 radius, the walkable interior is approximately
 * [0.6,9.4] x [0.6,9.4] — every fixture point below is chosen well clear of
 * that boundary unless a test is deliberately probing the edge.
 */
final class TestFixtures {
    private TestFixtures() {}

    static Geometry flatQuad() {
        return Geometry.newBuilder()
                .addVertices(0).addVertices(0).addVertices(0)
                .addVertices(10).addVertices(0).addVertices(0)
                .addVertices(10).addVertices(0).addVertices(10)
                .addVertices(0).addVertices(0).addVertices(10)
                .addTriangles(0).addTriangles(2).addTriangles(1)
                .addTriangles(0).addTriangles(3).addTriangles(2)
                .build();
    }

    static AgentConfig defaultAgent() {
        return AgentConfig.newBuilder()
                .setRadius(0.6f)
                .setHeight(2.0f)
                .setMaxSlopeDeg(45.0f)
                .setMaxClimb(0.9f)
                .setCellSize(0.3f)
                .setCellHeight(0.2f)
                .build();
    }

    static Vec3 vec3(float x, float y, float z) {
        return Vec3.newBuilder().setX(x).setY(y).setZ(z).build();
    }

    static NavMesh buildFlatQuadNavMesh(AxiomContext ax) {
        BuildNavMeshOutput out = BuildNavMesh.buildNavMesh(ax,
                BuildNavMeshInput.newBuilder().setGeometry(flatQuad()).setAgent(defaultAgent()).build());
        if (!out.getOk()) {
            throw new IllegalStateException("fixture navmesh build failed: " + out.getError());
        }
        return out.getNavmesh();
    }
}
