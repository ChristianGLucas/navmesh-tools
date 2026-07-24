package nodes;

import axiom.AxiomContext;
import com.google.protobuf.ByteString;
import gen.Messages.FindRandomPointAroundCircleInput;
import gen.Messages.FindRandomPointAroundCircleOutput;
import gen.Messages.NavMesh;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class FindRandomPointAroundCircleTest {

    @Test
    public void sameSeed_returnsIdenticalPoint_determinism() {
        AxiomContext ax = TestSupport.newContext();
        NavMesh navmesh = TestFixtures.buildFlatQuadNavMesh(ax);
        FindRandomPointAroundCircleInput req = FindRandomPointAroundCircleInput.newBuilder().setNavmesh(navmesh)
                .setCenter(TestFixtures.vec3(5, 0, 5)).setRadius(2f).setSeed(99).build();
        FindRandomPointAroundCircleOutput a = FindRandomPointAroundCircle.findRandomPointAroundCircle(ax, req);
        FindRandomPointAroundCircleOutput b = FindRandomPointAroundCircle.findRandomPointAroundCircle(ax, req);
        assertTrue(a.getOk() && a.getFound());
        assertTrue(b.getOk() && b.getFound());
        assertEquals(a.getPolyRef(), b.getPolyRef());
        assertEquals(a.getPoint().getX(), b.getPoint().getX(), 0.0);
        assertEquals(a.getPoint().getZ(), b.getPoint().getZ(), 0.0);
    }

    @Test
    public void sampledPoint_liesWithinRadiusOfCenter_handComputed() {
        AxiomContext ax = TestSupport.newContext();
        NavMesh navmesh = TestFixtures.buildFlatQuadNavMesh(ax);
        float radius = 1.5f;
        FindRandomPointAroundCircleOutput out = FindRandomPointAroundCircle.findRandomPointAroundCircle(ax,
                FindRandomPointAroundCircleInput.newBuilder().setNavmesh(navmesh)
                        .setCenter(TestFixtures.vec3(5, 0, 5)).setRadius(radius).setSeed(3).build());
        assertTrue(out.getOk() && out.getFound());
        double dx = out.getPoint().getX() - 5.0;
        double dz = out.getPoint().getZ() - 5.0;
        double dist = Math.sqrt(dx * dx + dz * dz);
        // The strict constraint clips each candidate polygon to its
        // intersection with a 12-sided dodecagon approximating the circle,
        // so the true bound is radius/cos(pi/12) (~3.5% over the nominal
        // radius) plus a small floating-point slack.
        double bound = radius / Math.cos(Math.PI / 12) + 0.05;
        assertTrue(dist <= bound, "sampled point " + dist + " units from center, expected <= " + bound);
    }

    @Test
    public void centerFarFromMesh_notFound_notError() {
        AxiomContext ax = TestSupport.newContext();
        NavMesh navmesh = TestFixtures.buildFlatQuadNavMesh(ax);
        FindRandomPointAroundCircleOutput out = FindRandomPointAroundCircle.findRandomPointAroundCircle(ax,
                FindRandomPointAroundCircleInput.newBuilder().setNavmesh(navmesh)
                        .setCenter(TestFixtures.vec3(-9999, 0, -9999)).setRadius(2f).setSeed(1).build());
        assertTrue(out.getOk(), out.getError());
        assertFalse(out.getFound());
    }

    @Test
    public void negativeRadius_returnsStructuredError_notCrash() {
        AxiomContext ax = TestSupport.newContext();
        NavMesh navmesh = TestFixtures.buildFlatQuadNavMesh(ax);
        FindRandomPointAroundCircleOutput out = FindRandomPointAroundCircle.findRandomPointAroundCircle(ax,
                FindRandomPointAroundCircleInput.newBuilder().setNavmesh(navmesh)
                        .setCenter(TestFixtures.vec3(5, 0, 5)).setRadius(-1f).setSeed(1).build());
        assertFalse(out.getOk());
        assertTrue(out.getError().startsWith("INVALID_INPUT"));
    }

    @Test
    public void malformedNavMeshBytes_returnsStructuredError_notCrash() {
        AxiomContext ax = TestSupport.newContext();
        NavMesh bad = NavMesh.newBuilder().setData(ByteString.copyFrom(new byte[] { 3 })).build();
        FindRandomPointAroundCircleOutput out = FindRandomPointAroundCircle.findRandomPointAroundCircle(ax,
                FindRandomPointAroundCircleInput.newBuilder().setNavmesh(bad).setCenter(TestFixtures.vec3(0, 0, 0))
                        .setRadius(1f).setSeed(1).build());
        assertFalse(out.getOk());
        assertTrue(out.getError().startsWith("INVALID_NAVMESH"));
    }
}
