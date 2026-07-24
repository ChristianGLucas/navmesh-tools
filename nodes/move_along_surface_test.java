package nodes;

import axiom.AxiomContext;
import com.google.protobuf.ByteString;
import gen.Messages.MoveAlongSurfaceInput;
import gen.Messages.MoveAlongSurfaceOutput;
import gen.Messages.NavMesh;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class MoveAlongSurfaceTest {

    @Test
    public void unobstructedInteriorMove_reachesEndExactly_handComputed() {
        AxiomContext ax = TestSupport.newContext();
        NavMesh navmesh = TestFixtures.buildFlatQuadNavMesh(ax);
        MoveAlongSurfaceOutput out = MoveAlongSurface.moveAlongSurface(ax, MoveAlongSurfaceInput.newBuilder()
                .setNavmesh(navmesh).setStart(TestFixtures.vec3(2, 0, 2)).setEnd(TestFixtures.vec3(7, 0, 3)).build());
        assertTrue(out.getOk(), out.getError());
        assertEquals(7.0, out.getResultPoint().getX(), 0.05);
        assertEquals(3.0, out.getResultPoint().getZ(), 0.05);
    }

    @Test
    public void moveTowardFarOutsidePoint_clipsNearWalkableBoundary_handComputed() {
        AxiomContext ax = TestSupport.newContext();
        NavMesh navmesh = TestFixtures.buildFlatQuadNavMesh(ax);
        MoveAlongSurfaceOutput out = MoveAlongSurface.moveAlongSurface(ax,
                MoveAlongSurfaceInput.newBuilder().setNavmesh(navmesh).setStart(TestFixtures.vec3(5, 0, 5))
                        .setEnd(TestFixtures.vec3(500, 0, 5)).build());
        assertTrue(out.getOk(), out.getError());
        // Clipped well short of x=500, near the ~9.4 eroded boundary.
        assertTrue(out.getResultPoint().getX() < 10.0f);
        assertTrue(out.getResultPoint().getX() > 8.0f);
    }

    @Test
    public void startFarFromMesh_returnsStructuredError_notCrash() {
        AxiomContext ax = TestSupport.newContext();
        NavMesh navmesh = TestFixtures.buildFlatQuadNavMesh(ax);
        MoveAlongSurfaceOutput out = MoveAlongSurface.moveAlongSurface(ax,
                MoveAlongSurfaceInput.newBuilder().setNavmesh(navmesh).setStart(TestFixtures.vec3(-9999, 0, -9999))
                        .setEnd(TestFixtures.vec3(5, 0, 5)).build());
        assertFalse(out.getOk());
        assertTrue(out.getError().startsWith("NO_POLY_NEAR_START"));
    }

    @Test
    public void malformedNavMeshBytes_returnsStructuredError_notCrash() {
        AxiomContext ax = TestSupport.newContext();
        NavMesh bad = NavMesh.newBuilder().setData(ByteString.copyFrom(new byte[] { 4 })).build();
        MoveAlongSurfaceOutput out = MoveAlongSurface.moveAlongSurface(ax, MoveAlongSurfaceInput.newBuilder()
                .setNavmesh(bad).setStart(TestFixtures.vec3(0, 0, 0)).setEnd(TestFixtures.vec3(1, 0, 1)).build());
        assertFalse(out.getOk());
        assertTrue(out.getError().startsWith("INVALID_NAVMESH"));
    }
}
