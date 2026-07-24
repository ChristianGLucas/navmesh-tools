package nodes;

import axiom.AxiomContext;
import com.google.protobuf.ByteString;
import gen.Messages.FindRandomPointInput;
import gen.Messages.FindRandomPointOutput;
import gen.Messages.NavMesh;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class FindRandomPointTest {

    @Test
    public void sameSeed_returnsIdenticalPoint_determinism() {
        AxiomContext ax = TestSupport.newContext();
        NavMesh navmesh = TestFixtures.buildFlatQuadNavMesh(ax);
        FindRandomPointInput req = FindRandomPointInput.newBuilder().setNavmesh(navmesh).setSeed(42).build();
        FindRandomPointOutput a = FindRandomPoint.findRandomPoint(ax, req);
        FindRandomPointOutput b = FindRandomPoint.findRandomPoint(ax, req);
        assertTrue(a.getOk(), a.getError());
        assertTrue(b.getOk(), b.getError());
        assertEquals(a.getPolyRef(), b.getPolyRef());
        assertEquals(a.getPoint().getX(), b.getPoint().getX(), 0.0);
        assertEquals(a.getPoint().getY(), b.getPoint().getY(), 0.0);
        assertEquals(a.getPoint().getZ(), b.getPoint().getZ(), 0.0);
    }

    @Test
    public void differentSeeds_typicallyReturnDifferentPoints() {
        AxiomContext ax = TestSupport.newContext();
        NavMesh navmesh = TestFixtures.buildFlatQuadNavMesh(ax);
        FindRandomPointOutput a = FindRandomPoint.findRandomPoint(ax,
                FindRandomPointInput.newBuilder().setNavmesh(navmesh).setSeed(1).build());
        FindRandomPointOutput b = FindRandomPoint.findRandomPoint(ax,
                FindRandomPointInput.newBuilder().setNavmesh(navmesh).setSeed(2).build());
        assertTrue(a.getOk() && b.getOk());
        assertNotEquals(a.getPoint().getX(), b.getPoint().getX());
    }

    @Test
    public void sampledPoint_liesWithinMeshBounds_handComputed() {
        AxiomContext ax = TestSupport.newContext();
        NavMesh navmesh = TestFixtures.buildFlatQuadNavMesh(ax);
        FindRandomPointOutput out = FindRandomPoint.findRandomPoint(ax,
                FindRandomPointInput.newBuilder().setNavmesh(navmesh).setSeed(7).build());
        assertTrue(out.getOk(), out.getError());
        // The walkable interior is eroded inward from the 10x10 quad — every
        // sample must lie within the un-eroded outer bound as a sanity check.
        assertTrue(out.getPoint().getX() >= 0f && out.getPoint().getX() <= 10f);
        assertTrue(out.getPoint().getZ() >= 0f && out.getPoint().getZ() <= 10f);
    }

    @Test
    public void malformedNavMeshBytes_returnsStructuredError_notCrash() {
        AxiomContext ax = TestSupport.newContext();
        NavMesh bad = NavMesh.newBuilder().setData(ByteString.copyFrom(new byte[] { 2, 2 })).build();
        FindRandomPointOutput out = FindRandomPoint.findRandomPoint(ax,
                FindRandomPointInput.newBuilder().setNavmesh(bad).setSeed(1).build());
        assertFalse(out.getOk());
        assertTrue(out.getError().startsWith("INVALID_NAVMESH"));
    }
}
