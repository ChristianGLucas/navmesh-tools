package nodes;

import axiom.AxiomContext;
import gen.Messages.FindStraightPathInput;
import gen.Messages.FindStraightPathOutput;

import org.recast4j.detour.NavMeshQuery;
import org.recast4j.detour.Result;
import org.recast4j.detour.StraightPathItem;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class FindStraightPath {

    /**
     * Straighten a FindPath polygon corridor into the minimal walkable line
     * of waypoint corners from start to end.
     */
    public static FindStraightPathOutput findStraightPath(AxiomContext ax, FindStraightPathInput input) {
        ax.log().info("FindStraightPath handling", Map.of());
        try {
            NavMeshQuery query = NavMeshUtil.queryFor(input.getNavmesh());

            float[] startPos = NavMeshUtil.toArr(input.getStart());
            float[] endPos = NavMeshUtil.toArr(input.getEnd());
            if (!NavMeshUtil.isFinite3(startPos) || !NavMeshUtil.isFinite3(endPos)) {
                return FindStraightPathOutput.newBuilder().setOk(false)
                        .setError("INVALID_INPUT: start/end must be finite").build();
            }
            if (input.getPolyRefsCount() == 0) {
                return FindStraightPathOutput.newBuilder().setOk(false)
                        .setError("INVALID_INPUT: poly_refs is empty — pass FindPath's poly_refs").build();
            }

            List<Long> path = new ArrayList<>(input.getPolyRefsCount());
            for (int i = 0; i < input.getPolyRefsCount(); i++) {
                path.add(input.getPolyRefs(i));
            }

            int maxStraightPath = Math.max(2, input.getPolyRefsCount() + 2);
            Result<List<StraightPathItem>> result = query.findStraightPath(startPos, endPos, path, maxStraightPath, 0);
            if (result.failed() || result.result == null) {
                return FindStraightPathOutput.newBuilder().setOk(false)
                        .setError("QUERY_FAILED: could not straighten the given poly_refs corridor").build();
            }

            FindStraightPathOutput.Builder out = FindStraightPathOutput.newBuilder().setOk(true);
            for (StraightPathItem item : result.result) {
                out.addWaypoints(NavMeshUtil.fromArr(item.getPos()));
            }
            return out.build();
        } catch (NavMeshUtil.NavOpException e) {
            return FindStraightPathOutput.newBuilder().setOk(false).setError(NavMeshUtil.toError(e)).build();
        } catch (Exception e) {
            return FindStraightPathOutput.newBuilder().setOk(false)
                    .setError("QUERY_FAILED: unexpected error: " + e.getMessage()).build();
        }
    }
}
