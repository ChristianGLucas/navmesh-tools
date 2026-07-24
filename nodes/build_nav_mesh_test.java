package nodes;

import axiom.AxiomContext;
import gen.Messages.AgentConfig;
import gen.Messages.BuildNavMeshInput;
import gen.Messages.BuildNavMeshOutput;
import gen.Messages.Geometry;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class BuildNavMeshTest {

    @Test
    public void flatQuad_buildsOneOrMorePolys_boundsMatchInput() {
        AxiomContext ax = TestSupport.newContext();
        BuildNavMeshOutput out = BuildNavMesh.buildNavMesh(ax,
                BuildNavMeshInput.newBuilder().setGeometry(TestFixtures.flatQuad()).setAgent(TestFixtures.defaultAgent())
                        .build());
        assertTrue(out.getOk(), out.getError());
        assertTrue(out.getNavmesh().getPolyCount() >= 1);
        assertTrue(out.getNavmesh().getVertCount() >= 3);
        assertFalse(out.getNavmesh().getData().isEmpty());
        // Recast's polymesh bounds are computed directly from the input geometry's
        // own bounding box (see RecastBuilderConfig(cfg, bmin, bmax)) — hand-known
        // for our 10x10 quad at y=0.
        assertEquals(0.0, out.getNavmesh().getBoundsMin().getX(), 0.5);
        assertEquals(0.0, out.getNavmesh().getBoundsMin().getZ(), 0.5);
        assertEquals(10.0, out.getNavmesh().getBoundsMax().getX(), 0.5);
        assertEquals(10.0, out.getNavmesh().getBoundsMax().getZ(), 0.5);
    }

    @Test
    public void sameGeometryAndAgent_buildsByteIdenticalNavMesh_determinism() {
        AxiomContext ax = TestSupport.newContext();
        BuildNavMeshInput input = BuildNavMeshInput.newBuilder().setGeometry(TestFixtures.flatQuad())
                .setAgent(TestFixtures.defaultAgent()).build();
        BuildNavMeshOutput a = BuildNavMesh.buildNavMesh(ax, input);
        BuildNavMeshOutput b = BuildNavMesh.buildNavMesh(ax, input);
        assertTrue(a.getOk());
        assertTrue(b.getOk());
        assertEquals(a.getNavmesh().getData(), b.getNavmesh().getData());
        assertEquals(a.getNavmesh().getPolyCount(), b.getNavmesh().getPolyCount());
        assertEquals(a.getNavmesh().getVertCount(), b.getNavmesh().getVertCount());
    }

    @Test
    public void emptyGeometry_returnsStructuredError_notCrash() {
        AxiomContext ax = TestSupport.newContext();
        BuildNavMeshOutput out = BuildNavMesh.buildNavMesh(ax,
                BuildNavMeshInput.newBuilder().setGeometry(Geometry.newBuilder().build())
                        .setAgent(TestFixtures.defaultAgent()).build());
        assertFalse(out.getOk());
        assertTrue(out.getError().startsWith("EMPTY_GEOMETRY"));
    }

    @Test
    public void triangleIndexOutOfRange_returnsStructuredError_notCrash() {
        AxiomContext ax = TestSupport.newContext();
        Geometry bad = Geometry.newBuilder()
                .addVertices(0).addVertices(0).addVertices(0)
                .addVertices(1).addVertices(0).addVertices(0)
                .addVertices(1).addVertices(0).addVertices(1)
                // triangle references vertex index 9, but only 3 vertices (indices 0-2) exist
                .addTriangles(0).addTriangles(1).addTriangles(9)
                .build();
        BuildNavMeshOutput out = BuildNavMesh.buildNavMesh(ax,
                BuildNavMeshInput.newBuilder().setGeometry(bad).setAgent(TestFixtures.defaultAgent()).build());
        assertFalse(out.getOk());
        assertTrue(out.getError().startsWith("INVALID_GEOMETRY"));
    }

    @Test
    public void nonFiniteVertex_returnsStructuredError_notCrash() {
        AxiomContext ax = TestSupport.newContext();
        Geometry bad = Geometry.newBuilder()
                .addVertices(Float.NaN).addVertices(0).addVertices(0)
                .addVertices(1).addVertices(0).addVertices(0)
                .addVertices(1).addVertices(0).addVertices(1)
                .addTriangles(0).addTriangles(2).addTriangles(1)
                .build();
        BuildNavMeshOutput out = BuildNavMesh.buildNavMesh(ax,
                BuildNavMeshInput.newBuilder().setGeometry(bad).setAgent(TestFixtures.defaultAgent()).build());
        assertFalse(out.getOk());
        assertTrue(out.getError().startsWith("INVALID_GEOMETRY"));
    }

    @Test
    public void malformedTrianglesLength_returnsStructuredError_notCrash() {
        AxiomContext ax = TestSupport.newContext();
        Geometry bad = Geometry.newBuilder()
                .addVertices(0).addVertices(0).addVertices(0)
                .addVertices(1).addVertices(0).addVertices(0)
                .addVertices(1).addVertices(0).addVertices(1)
                .addTriangles(0).addTriangles(1) // not a multiple of 3
                .build();
        BuildNavMeshOutput out = BuildNavMesh.buildNavMesh(ax,
                BuildNavMeshInput.newBuilder().setGeometry(bad).setAgent(TestFixtures.defaultAgent()).build());
        assertFalse(out.getOk());
        assertTrue(out.getError().startsWith("INVALID_GEOMETRY"));
    }

    @Test
    public void nonPositiveRadius_returnsStructuredError_notCrash() {
        AxiomContext ax = TestSupport.newContext();
        AgentConfig bad = TestFixtures.defaultAgent().toBuilder().setRadius(0f).build();
        BuildNavMeshOutput out = BuildNavMesh.buildNavMesh(ax,
                BuildNavMeshInput.newBuilder().setGeometry(TestFixtures.flatQuad()).setAgent(bad).build());
        assertFalse(out.getOk());
        assertTrue(out.getError().startsWith("INVALID_AGENT_CONFIG"));
    }

    @Test
    public void agentTooBigForGeometry_returnsNoWalkableSurfaceError_notCrash() {
        AxiomContext ax = TestSupport.newContext();
        // Radius 100 erodes clean through a 10x10 quad — nothing walkable remains.
        AgentConfig huge = TestFixtures.defaultAgent().toBuilder().setRadius(100f).build();
        BuildNavMeshOutput out = BuildNavMesh.buildNavMesh(ax,
                BuildNavMeshInput.newBuilder().setGeometry(TestFixtures.flatQuad()).setAgent(huge).build());
        assertFalse(out.getOk());
        assertTrue(out.getError().startsWith("NO_WALKABLE_SURFACE"));
    }
}
