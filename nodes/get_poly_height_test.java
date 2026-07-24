package nodes;

import axiom.AxiomContext;
import com.google.protobuf.ByteString;
import gen.Messages.GetPolyHeightInput;
import gen.Messages.GetPolyHeightOutput;
import gen.Messages.NavMesh;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class GetPolyHeightTest {

    @Test
    public void interiorPoint_onFlatQuadAtYZero_heightIsZero_handComputed() {
        AxiomContext ax = TestSupport.newContext();
        NavMesh navmesh = TestFixtures.buildFlatQuadNavMesh(ax);
        GetPolyHeightOutput out = GetPolyHeight.getPolyHeight(ax,
                GetPolyHeightInput.newBuilder().setNavmesh(navmesh).setPoint(TestFixtures.vec3(5, 3, 5)).build());
        assertTrue(out.getOk(), out.getError());
        assertTrue(out.getFound());
        assertEquals(0.0, out.getHeight(), 0.3);
    }

    @Test
    public void pointFarFromMesh_notFound_notError() {
        AxiomContext ax = TestSupport.newContext();
        NavMesh navmesh = TestFixtures.buildFlatQuadNavMesh(ax);
        GetPolyHeightOutput out = GetPolyHeight.getPolyHeight(ax, GetPolyHeightInput.newBuilder().setNavmesh(navmesh)
                .setPoint(TestFixtures.vec3(-9999, 0, -9999)).build());
        assertTrue(out.getOk(), out.getError());
        assertFalse(out.getFound());
    }

    @Test
    public void malformedNavMeshBytes_returnsStructuredError_notCrash() {
        AxiomContext ax = TestSupport.newContext();
        NavMesh bad = NavMesh.newBuilder().setData(ByteString.copyFrom(new byte[] { 5 })).build();
        GetPolyHeightOutput out = GetPolyHeight.getPolyHeight(ax,
                GetPolyHeightInput.newBuilder().setNavmesh(bad).setPoint(TestFixtures.vec3(0, 0, 0)).build());
        assertFalse(out.getOk());
        assertTrue(out.getError().startsWith("INVALID_NAVMESH"));
    }
}
