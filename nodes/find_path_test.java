package nodes;

import axiom.AxiomContext;
import com.google.protobuf.ByteString;
import gen.Messages.FindPathInput;
import gen.Messages.FindPathOutput;
import gen.Messages.NavMesh;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class FindPathTest {

    @Test
    public void interiorPoints_onFlatQuad_findsConnectedPath_handComputed() {
        AxiomContext ax = TestSupport.newContext();
        NavMesh navmesh = TestFixtures.buildFlatQuadNavMesh(ax);
        FindPathOutput out = FindPath.findPath(ax, FindPathInput.newBuilder().setNavmesh(navmesh)
                .setStart(TestFixtures.vec3(2, 0, 2)).setEnd(TestFixtures.vec3(8, 0, 8)).build());
        assertTrue(out.getOk(), out.getError());
        assertTrue(out.getPathFound());
        assertFalse(out.getPartial());
        assertTrue(out.getPolyRefsCount() >= 1);
    }

    @Test
    public void pointsFarOutsideMesh_returnsPathNotFound_notError() {
        AxiomContext ax = TestSupport.newContext();
        NavMesh navmesh = TestFixtures.buildFlatQuadNavMesh(ax);
        FindPathOutput out = FindPath.findPath(ax, FindPathInput.newBuilder().setNavmesh(navmesh)
                .setStart(TestFixtures.vec3(-1000, 0, -1000)).setEnd(TestFixtures.vec3(2000, 0, 2000)).build());
        assertTrue(out.getOk(), out.getError());
        assertFalse(out.getPathFound());
    }

    @Test
    public void sameQueryTwice_returnsIdenticalCorridor_determinism() {
        AxiomContext ax = TestSupport.newContext();
        NavMesh navmesh = TestFixtures.buildFlatQuadNavMesh(ax);
        FindPathInput req = FindPathInput.newBuilder().setNavmesh(navmesh).setStart(TestFixtures.vec3(1, 0, 1))
                .setEnd(TestFixtures.vec3(9, 0, 9)).build();
        FindPathOutput a = FindPath.findPath(ax, req);
        FindPathOutput b = FindPath.findPath(ax, req);
        assertEquals(a.getPolyRefsList(), b.getPolyRefsList());
    }

    @Test
    public void malformedNavMeshBytes_returnsStructuredError_notCrash() {
        AxiomContext ax = TestSupport.newContext();
        NavMesh bad = NavMesh.newBuilder().setData(ByteString.copyFrom(new byte[] { 1, 2, 3, 4 })).build();
        FindPathOutput out = FindPath.findPath(ax, FindPathInput.newBuilder().setNavmesh(bad)
                .setStart(TestFixtures.vec3(0, 0, 0)).setEnd(TestFixtures.vec3(1, 0, 1)).build());
        assertFalse(out.getOk());
        assertTrue(out.getError().startsWith("INVALID_NAVMESH"));
    }

    @Test
    public void nonFiniteStart_returnsStructuredError_notCrash() {
        AxiomContext ax = TestSupport.newContext();
        NavMesh navmesh = TestFixtures.buildFlatQuadNavMesh(ax);
        FindPathOutput out = FindPath.findPath(ax, FindPathInput.newBuilder().setNavmesh(navmesh)
                .setStart(TestFixtures.vec3(Float.NaN, 0, 0)).setEnd(TestFixtures.vec3(1, 0, 1)).build());
        assertFalse(out.getOk());
        assertTrue(out.getError().startsWith("INVALID_INPUT"));
    }
}
