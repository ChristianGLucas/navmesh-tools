package nodes;

import axiom.AxiomContext;
import gen.Messages.RaycastInput;
import gen.Messages.RaycastOutput;

import org.recast4j.detour.DefaultQueryFilter;
import org.recast4j.detour.FindNearestPolyResult;
import org.recast4j.detour.NavMeshQuery;
import org.recast4j.detour.RaycastHit;
import org.recast4j.detour.Result;

import java.util.Map;

public class Raycast {

    /**
     * Cast a ray across the navmesh's walkable surface from start toward
     * end; reports the first wall/edge hit, or that end is fully reachable.
     */
    public static RaycastOutput raycast(AxiomContext ax, RaycastInput input) {
        ax.log().info("Raycast handling", Map.of());
        try {
            NavMeshQuery query = NavMeshUtil.queryFor(input.getNavmesh());
            DefaultQueryFilter filter = NavMeshUtil.defaultFilter();

            float[] startPos = NavMeshUtil.toArr(input.getStart());
            float[] endPos = NavMeshUtil.toArr(input.getEnd());
            if (!NavMeshUtil.isFinite3(startPos) || !NavMeshUtil.isFinite3(endPos)) {
                return RaycastOutput.newBuilder().setOk(false).setError("INVALID_INPUT: start/end must be finite")
                        .build();
            }

            float[] halfExtents = NavMeshUtil.defaultHalfExtents(input.getNavmesh(), null);
            Result<FindNearestPolyResult> startNear = query.findNearestPoly(startPos, halfExtents, filter);
            if (startNear.failed() || startNear.result.getNearestRef() == 0) {
                return RaycastOutput.newBuilder().setOk(false)
                        .setError("NO_POLY_NEAR_START: start point is not near any navmesh polygon").build();
            }
            long startRef = startNear.result.getNearestRef();

            Result<RaycastHit> hitResult = query.raycast(startRef, startPos, endPos, filter, 0, 0);
            if (hitResult.failed() || hitResult.result == null) {
                return RaycastOutput.newBuilder().setOk(false).setError("QUERY_FAILED: raycast did not complete")
                        .build();
            }

            RaycastHit hit = hitResult.result;
            boolean didHit = Float.isFinite(hit.t) && hit.t < 1.0f;
            float fraction = didHit ? Math.max(0f, Math.min(1f, hit.t)) : 1.0f;
            float[] hitPoint = new float[3];
            for (int i = 0; i < 3; i++) {
                hitPoint[i] = startPos[i] + (endPos[i] - startPos[i]) * fraction;
            }

            return RaycastOutput.newBuilder().setOk(true).setHit(didHit).setHitPoint(NavMeshUtil.fromArr(hitPoint))
                    .setHitFraction(fraction).build();
        } catch (NavMeshUtil.NavOpException e) {
            return RaycastOutput.newBuilder().setOk(false).setError(NavMeshUtil.toError(e)).build();
        } catch (Exception e) {
            return RaycastOutput.newBuilder().setOk(false)
                    .setError("QUERY_FAILED: unexpected error: " + e.getMessage()).build();
        }
    }
}
