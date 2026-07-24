package nodes;

import axiom.AxiomContext;
import com.google.protobuf.ByteString;
import gen.Messages.FindNearestPolyInput;
import gen.Messages.FindNearestPolyOutput;
import gen.Messages.NavMesh;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class FindNearestPolyTest {

    @Test
    public void interiorPoint_snapsToItself_handComputed() {
        AxiomContext ax = TestSupport.newContext();
        NavMesh navmesh = TestFixtures.buildFlatQuadNavMesh(ax);
        FindNearestPolyOutput out = FindNearestPoly.findNearestPoly(ax, FindNearestPolyInput.newBuilder()
                .setNavmesh(navmesh).setPoint(TestFixtures.vec3(5, 0, 5)).build());
        assertTrue(out.getOk(), out.getError());
        assertTrue(out.getFound());
        assertNotEquals(0L, out.getPolyRef());
        assertEquals(5.0, out.getNearestPoint().getX(), 0.1);
        assertEquals(5.0, out.getNearestPoint().getZ(), 0.1);
        assertEquals(0.0, out.getNearestPoint().getY(), 0.3);
    }

    @Test
    public void pointFarFromMesh_notFound_notError() {
        AxiomContext ax = TestSupport.newContext();
        NavMesh navmesh = TestFixtures.buildFlatQuadNavMesh(ax);
        FindNearestPolyOutput out = FindNearestPoly.findNearestPoly(ax, FindNearestPolyInput.newBuilder()
                .setNavmesh(navmesh).setPoint(TestFixtures.vec3(-9999, 0, -9999)).build());
        assertTrue(out.getOk(), out.getError());
        assertFalse(out.getFound());
    }

    @Test
    public void malformedNavMeshBytes_returnsStructuredError_notCrash() {
        AxiomContext ax = TestSupport.newContext();
        NavMesh bad = NavMesh.newBuilder().setData(ByteString.copyFrom(new byte[] { 1 })).build();
        FindNearestPolyOutput out = FindNearestPoly.findNearestPoly(ax,
                FindNearestPolyInput.newBuilder().setNavmesh(bad).setPoint(TestFixtures.vec3(0, 0, 0)).build());
        assertFalse(out.getOk());
        assertTrue(out.getError().startsWith("INVALID_NAVMESH"));
    }
}
