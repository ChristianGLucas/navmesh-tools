package nodes;

import axiom.AxiomContext;
import gen.Messages.AgentConfig;
import gen.Messages.BuildAndFindPathInput;
import gen.Messages.BuildAndFindPathOutput;
import gen.Messages.Geometry;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class BuildAndFindPathTest {

    @Test
    public void interiorPoints_onFlatQuad_isExactlyTheStraightSegment_handComputed() {
        AxiomContext ax = TestSupport.newContext();
        BuildAndFindPathOutput out = BuildAndFindPath.buildAndFindPath(ax,
                BuildAndFindPathInput.newBuilder().setGeometry(TestFixtures.flatQuad())
                        .setAgent(TestFixtures.defaultAgent()).setStart(TestFixtures.vec3(2, 0, 2))
                        .setEnd(TestFixtures.vec3(8, 0, 8)).build());
        assertTrue(out.getOk(), out.getError());
        assertTrue(out.getPathFound());
        assertFalse(out.getPartial());
        assertEquals(2, out.getWaypointsCount());
        assertEquals(2.0, out.getWaypoints(0).getX(), 1e-4);
        assertEquals(8.0, out.getWaypoints(1).getX(), 1e-4);
    }

    @Test
    public void pointsFarOutsideMesh_returnsPathNotFound_notError() {
        AxiomContext ax = TestSupport.newContext();
        BuildAndFindPathOutput out = BuildAndFindPath.buildAndFindPath(ax,
                BuildAndFindPathInput.newBuilder().setGeometry(TestFixtures.flatQuad())
                        .setAgent(TestFixtures.defaultAgent()).setStart(TestFixtures.vec3(-9999, 0, -9999))
                        .setEnd(TestFixtures.vec3(9999, 0, 9999)).build());
        assertTrue(out.getOk(), out.getError());
        assertFalse(out.getPathFound());
    }

    @Test
    public void emptyGeometry_returnsStructuredError_notCrash() {
        AxiomContext ax = TestSupport.newContext();
        BuildAndFindPathOutput out = BuildAndFindPath.buildAndFindPath(ax,
                BuildAndFindPathInput.newBuilder().setGeometry(Geometry.newBuilder().build())
                        .setAgent(TestFixtures.defaultAgent()).setStart(TestFixtures.vec3(0, 0, 0))
                        .setEnd(TestFixtures.vec3(1, 0, 1)).build());
        assertFalse(out.getOk());
        assertTrue(out.getError().startsWith("EMPTY_GEOMETRY"));
    }

    @Test
    public void invalidAgentConfig_returnsStructuredError_notCrash() {
        AxiomContext ax = TestSupport.newContext();
        AgentConfig bad = TestFixtures.defaultAgent().toBuilder().setHeight(-1f).build();
        BuildAndFindPathOutput out = BuildAndFindPath.buildAndFindPath(ax,
                BuildAndFindPathInput.newBuilder().setGeometry(TestFixtures.flatQuad()).setAgent(bad)
                        .setStart(TestFixtures.vec3(0, 0, 0)).setEnd(TestFixtures.vec3(1, 0, 1)).build());
        assertFalse(out.getOk());
        assertTrue(out.getError().startsWith("INVALID_AGENT_CONFIG"));
    }
}
