package nodes;

import axiom.AxiomContext;
import com.google.protobuf.ByteString;
import gen.Messages.NavMesh;
import gen.Messages.RaycastInput;
import gen.Messages.RaycastOutput;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class RaycastTest {

    @Test
    public void unobstructedInteriorSegment_reachesEndFully_handComputed() {
        AxiomContext ax = TestSupport.newContext();
        NavMesh navmesh = TestFixtures.buildFlatQuadNavMesh(ax);
        RaycastOutput out = Raycast.raycast(ax, RaycastInput.newBuilder().setNavmesh(navmesh)
                .setStart(TestFixtures.vec3(2, 0, 5)).setEnd(TestFixtures.vec3(8, 0, 5)).build());
        assertTrue(out.getOk(), out.getError());
        assertFalse(out.getHit());
        assertEquals(1.0, out.getHitFraction(), 1e-4);
        assertEquals(8.0, out.getHitPoint().getX(), 1e-4);
        assertEquals(5.0, out.getHitPoint().getZ(), 1e-4);
    }

    @Test
    public void rayAcrossFarEdge_stopsAtWalkableBoundary_handComputed() {
        AxiomContext ax = TestSupport.newContext();
        NavMesh navmesh = TestFixtures.buildFlatQuadNavMesh(ax);
        // The quad's walkable interior ends around x=9.4 (10 minus the 0.6
        // agent-radius erosion). Casting from x=5 to x=20 must be blocked
        // well before reaching x=20.
        RaycastOutput out = Raycast.raycast(ax, RaycastInput.newBuilder().setNavmesh(navmesh)
                .setStart(TestFixtures.vec3(5, 0, 5)).setEnd(TestFixtures.vec3(20, 0, 5)).build());
        assertTrue(out.getOk(), out.getError());
        assertTrue(out.getHit());
        assertTrue(out.getHitFraction() < 1.0f);
        assertTrue(out.getHitPoint().getX() > 8.0f && out.getHitPoint().getX() < 10.0f,
                "expected hit near the eroded boundary (~9.4), got " + out.getHitPoint().getX());
        assertEquals(5.0, out.getHitPoint().getZ(), 0.5);
    }

    @Test
    public void startFarFromMesh_returnsStructuredError_notCrash() {
        AxiomContext ax = TestSupport.newContext();
        NavMesh navmesh = TestFixtures.buildFlatQuadNavMesh(ax);
        RaycastOutput out = Raycast.raycast(ax, RaycastInput.newBuilder().setNavmesh(navmesh)
                .setStart(TestFixtures.vec3(-9999, 0, -9999)).setEnd(TestFixtures.vec3(5, 0, 5)).build());
        assertFalse(out.getOk());
        assertTrue(out.getError().startsWith("NO_POLY_NEAR_START"));
    }

    @Test
    public void malformedNavMeshBytes_returnsStructuredError_notCrash() {
        AxiomContext ax = TestSupport.newContext();
        NavMesh bad = NavMesh.newBuilder().setData(ByteString.copyFrom(new byte[] { 7, 7 })).build();
        RaycastOutput out = Raycast.raycast(ax, RaycastInput.newBuilder().setNavmesh(bad)
                .setStart(TestFixtures.vec3(0, 0, 0)).setEnd(TestFixtures.vec3(1, 0, 1)).build());
        assertFalse(out.getOk());
        assertTrue(out.getError().startsWith("INVALID_NAVMESH"));
    }
}
