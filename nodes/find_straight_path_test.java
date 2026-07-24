package nodes;

import axiom.AxiomContext;
import com.google.protobuf.ByteString;
import gen.Messages.FindPathInput;
import gen.Messages.FindPathOutput;
import gen.Messages.FindStraightPathInput;
import gen.Messages.FindStraightPathOutput;
import gen.Messages.NavMesh;
import gen.Messages.Vec3;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class FindStraightPathTest {

    @Test
    public void interiorPointsOnConvexFlatQuad_isExactlyTheStraightSegment_handComputed() {
        AxiomContext ax = TestSupport.newContext();
        NavMesh navmesh = TestFixtures.buildFlatQuadNavMesh(ax);
        Vec3 start = TestFixtures.vec3(2, 0, 2);
        Vec3 end = TestFixtures.vec3(8, 0, 8);

        FindPathOutput path = FindPath.findPath(ax,
                FindPathInput.newBuilder().setNavmesh(navmesh).setStart(start).setEnd(end).build());
        assertTrue(path.getOk(), path.getError());
        assertTrue(path.getPathFound());

        FindStraightPathOutput out = FindStraightPath.findStraightPath(ax, FindStraightPathInput.newBuilder()
                .setNavmesh(navmesh).setStart(start).setEnd(end).addAllPolyRefs(path.getPolyRefsList()).build());
        assertTrue(out.getOk(), out.getError());

        // On a single flat convex polygon the straightened path is exactly
        // the two endpoints — no intermediate corners to bend around.
        assertEquals(2, out.getWaypointsCount());
        assertEquals(2.0, out.getWaypoints(0).getX(), 1e-4);
        assertEquals(2.0, out.getWaypoints(0).getZ(), 1e-4);
        assertEquals(8.0, out.getWaypoints(1).getX(), 1e-4);
        assertEquals(8.0, out.getWaypoints(1).getZ(), 1e-4);

        // Hand-computed Euclidean length: sqrt(6^2 + 6^2).
        double dx = out.getWaypoints(1).getX() - out.getWaypoints(0).getX();
        double dz = out.getWaypoints(1).getZ() - out.getWaypoints(0).getZ();
        double length = Math.sqrt(dx * dx + dz * dz);
        assertEquals(Math.sqrt(72.0), length, 1e-3);
    }

    @Test
    public void emptyPolyRefs_returnsStructuredError_notCrash() {
        AxiomContext ax = TestSupport.newContext();
        NavMesh navmesh = TestFixtures.buildFlatQuadNavMesh(ax);
        FindStraightPathOutput out = FindStraightPath.findStraightPath(ax,
                FindStraightPathInput.newBuilder().setNavmesh(navmesh).setStart(TestFixtures.vec3(1, 0, 1))
                        .setEnd(TestFixtures.vec3(2, 0, 2)).build());
        assertFalse(out.getOk());
        assertTrue(out.getError().startsWith("INVALID_INPUT"));
    }

    @Test
    public void malformedNavMeshBytes_returnsStructuredError_notCrash() {
        AxiomContext ax = TestSupport.newContext();
        NavMesh bad = NavMesh.newBuilder().setData(ByteString.copyFrom(new byte[] { 9, 9, 9 })).build();
        FindStraightPathOutput out = FindStraightPath.findStraightPath(ax,
                FindStraightPathInput.newBuilder().setNavmesh(bad).setStart(TestFixtures.vec3(0, 0, 0))
                        .setEnd(TestFixtures.vec3(1, 0, 1)).addPolyRefs(1L).build());
        assertFalse(out.getOk());
        assertTrue(out.getError().startsWith("INVALID_NAVMESH"));
    }
}
