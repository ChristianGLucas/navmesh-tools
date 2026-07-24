package nodes;

import axiom.AxiomContext;
import gen.Messages.BuildAndFindPathInput;
import gen.Messages.BuildAndFindPathOutput;

import org.recast4j.detour.DefaultQueryFilter;
import org.recast4j.detour.FindNearestPolyResult;
import org.recast4j.detour.NavMeshQuery;
import org.recast4j.detour.Result;
import org.recast4j.detour.Status;
import org.recast4j.detour.StraightPathItem;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class BuildAndFindPath {

    /**
     * One-shot convenience: build a navmesh from geometry+agent and
     * immediately find the straightened walkable path between two points.
     */
    public static BuildAndFindPathOutput buildAndFindPath(AxiomContext ax, BuildAndFindPathInput input) {
        ax.log().info("BuildAndFindPath handling", Map.of());
        try {
            NavMeshUtil.Built built = NavMeshUtil.buildNavMesh(input.getGeometry(), input.getAgent());
            NavMeshQuery query = new NavMeshQuery(built.navMesh);
            DefaultQueryFilter filter = NavMeshUtil.defaultFilter();

            float[] startPos = NavMeshUtil.toArr(input.getStart());
            float[] endPos = NavMeshUtil.toArr(input.getEnd());
            if (!NavMeshUtil.isFinite3(startPos) || !NavMeshUtil.isFinite3(endPos)) {
                return BuildAndFindPathOutput.newBuilder().setOk(false)
                        .setError("INVALID_INPUT: start/end must be finite").build();
            }

            float radius = input.getAgent().getRadius();
            float height = input.getAgent().getHeight();
            float hx = Math.max(radius * 4f, 0.5f);
            float hy = Math.max(height * 2f, 0.5f);
            float[] halfExtents = new float[] { hx, hy, hx };

            Result<FindNearestPolyResult> startNear = query.findNearestPoly(startPos, halfExtents, filter);
            Result<FindNearestPolyResult> endNear = query.findNearestPoly(endPos, halfExtents, filter);
            if (startNear.failed() || endNear.failed() || startNear.result.getNearestRef() == 0
                    || endNear.result.getNearestRef() == 0) {
                return BuildAndFindPathOutput.newBuilder().setOk(true).setPathFound(false).build();
            }

            Result<List<Long>> pathResult = query.findPath(startNear.result.getNearestRef(),
                    endNear.result.getNearestRef(), startPos, endPos, filter);
            if (pathResult.failed() || pathResult.result == null || pathResult.result.isEmpty()) {
                return BuildAndFindPathOutput.newBuilder().setOk(true).setPathFound(false).build();
            }

            int maxStraightPath = Math.max(2, pathResult.result.size() + 2);
            Result<List<StraightPathItem>> straight = query.findStraightPath(startPos, endPos, pathResult.result,
                    maxStraightPath, 0);
            if (straight.failed() || straight.result == null) {
                return BuildAndFindPathOutput.newBuilder().setOk(true).setPathFound(false).build();
            }

            BuildAndFindPathOutput.Builder out = BuildAndFindPathOutput.newBuilder().setOk(true).setPathFound(true)
                    .setPartial(pathResult.status == Status.PARTIAL_RESULT);
            for (StraightPathItem item : straight.result) {
                out.addWaypoints(NavMeshUtil.fromArr(item.getPos()));
            }
            return out.build();
        } catch (NavMeshUtil.NavOpException e) {
            return BuildAndFindPathOutput.newBuilder().setOk(false).setError(NavMeshUtil.toError(e)).build();
        } catch (Exception e) {
            return BuildAndFindPathOutput.newBuilder().setOk(false)
                    .setError("BUILD_FAILED: unexpected error: " + e.getMessage()).build();
        }
    }
}
