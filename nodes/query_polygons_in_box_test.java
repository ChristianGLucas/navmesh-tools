package nodes;

import axiom.AxiomContext;
import com.google.protobuf.ByteString;
import gen.Messages.NavMesh;
import gen.Messages.QueryPolygonsInBoxInput;
import gen.Messages.QueryPolygonsInBoxOutput;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class QueryPolygonsInBoxTest {

    @Test
    public void boxCoveringWholeMesh_returnsAllPolys_handComputed() {
        AxiomContext ax = TestSupport.newContext();
        NavMesh navmesh = TestFixtures.buildFlatQuadNavMesh(ax);
        QueryPolygonsInBoxOutput out = QueryPolygonsInBox.queryPolygonsInBox(ax,
                QueryPolygonsInBoxInput.newBuilder().setNavmesh(navmesh).setCenter(TestFixtures.vec3(5, 0, 5))
                        .setHalfExtents(TestFixtures.vec3(20, 20, 20)).build());
        assertTrue(out.getOk(), out.getError());
        assertEquals(navmesh.getPolyCount(), out.getPolyRefsCount());
    }

    @Test
    public void boxFarFromMesh_returnsEmpty_notError() {
        AxiomContext ax = TestSupport.newContext();
        NavMesh navmesh = TestFixtures.buildFlatQuadNavMesh(ax);
        QueryPolygonsInBoxOutput out = QueryPolygonsInBox.queryPolygonsInBox(ax,
                QueryPolygonsInBoxInput.newBuilder().setNavmesh(navmesh).setCenter(TestFixtures.vec3(-9999, 0, -9999))
                        .setHalfExtents(TestFixtures.vec3(1, 1, 1)).build());
        assertTrue(out.getOk(), out.getError());
        assertEquals(0, out.getPolyRefsCount());
    }

    @Test
    public void malformedNavMeshBytes_returnsStructuredError_notCrash() {
        AxiomContext ax = TestSupport.newContext();
        NavMesh bad = NavMesh.newBuilder().setData(ByteString.copyFrom(new byte[] { 6 })).build();
        QueryPolygonsInBoxOutput out = QueryPolygonsInBox.queryPolygonsInBox(ax,
                QueryPolygonsInBoxInput.newBuilder().setNavmesh(bad).setCenter(TestFixtures.vec3(0, 0, 0)).build());
        assertFalse(out.getOk());
        assertTrue(out.getError().startsWith("INVALID_NAVMESH"));
    }
}
