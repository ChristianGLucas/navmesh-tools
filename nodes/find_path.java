package nodes;

import axiom.AxiomContext;
import gen.Messages.FindPathInput;
import gen.Messages.FindPathOutput;

import org.recast4j.detour.DefaultQueryFilter;
import org.recast4j.detour.FindNearestPolyResult;
import org.recast4j.detour.NavMeshQuery;
import org.recast4j.detour.Result;
import org.recast4j.detour.Status;

import java.util.List;
import java.util.Map;

public class FindPath {

    /**
     * Find the shortest polygon-reference corridor across a built navmesh
     * between two points (each snapped to its nearest polygon first).
     */
    public static FindPathOutput findPath(AxiomContext ax, FindPathInput input) {
        ax.log().info("FindPath handling", Map.of());
        try {
            NavMeshQuery query = NavMeshUtil.queryFor(input.getNavmesh());
            DefaultQueryFilter filter = NavMeshUtil.defaultFilter();

            float[] startPos = NavMeshUtil.toArr(input.getStart());
            float[] endPos = NavMeshUtil.toArr(input.getEnd());
            if (!NavMeshUtil.isFinite3(startPos) || !NavMeshUtil.isFinite3(endPos)) {
                return FindPathOutput.newBuilder().setOk(false).setError("INVALID_INPUT: start/end must be finite")
                        .build();
            }

            float[] halfExtents = NavMeshUtil.defaultHalfExtents(input.getNavmesh(), null);
            Result<FindNearestPolyResult> startNear = query.findNearestPoly(startPos, halfExtents, filter);
            Result<FindNearestPolyResult> endNear = query.findNearestPoly(endPos, halfExtents, filter);

            if (startNear.failed() || endNear.failed() || startNear.result.getNearestRef() == 0
                    || endNear.result.getNearestRef() == 0) {
                return FindPathOutput.newBuilder().setOk(true).setPathFound(false).build();
            }

            long startRef = startNear.result.getNearestRef();
            long endRef = endNear.result.getNearestRef();

            Result<List<Long>> pathResult = query.findPath(startRef, endRef, startPos, endPos, filter);
            if (pathResult.failed() || pathResult.result == null || pathResult.result.isEmpty()) {
                return FindPathOutput.newBuilder().setOk(true).setPathFound(false).build();
            }

            FindPathOutput.Builder out = FindPathOutput.newBuilder().setOk(true).setPathFound(true)
                    .setPartial(pathResult.status == Status.PARTIAL_RESULT);
            for (long ref : pathResult.result) {
                out.addPolyRefs(ref);
            }
            return out.build();
        } catch (NavMeshUtil.NavOpException e) {
            return FindPathOutput.newBuilder().setOk(false).setError(NavMeshUtil.toError(e)).build();
        } catch (Exception e) {
            return FindPathOutput.newBuilder().setOk(false)
                    .setError("QUERY_FAILED: unexpected error: " + e.getMessage()).build();
        }
    }
}
