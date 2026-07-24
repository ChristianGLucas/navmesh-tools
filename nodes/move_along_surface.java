package nodes;

import axiom.AxiomContext;
import gen.Messages.MoveAlongSurfaceInput;
import gen.Messages.MoveAlongSurfaceOutput;

import org.recast4j.detour.DefaultQueryFilter;
import org.recast4j.detour.FindNearestPolyResult;
import org.recast4j.detour.MoveAlongSurfaceResult;
import org.recast4j.detour.NavMeshQuery;
import org.recast4j.detour.Result;

import java.util.Map;

public class MoveAlongSurface {

    /**
     * Slide from start toward end constrained to the walkable surface —
     * clipped at walls/edges rather than crossing them.
     */
    public static MoveAlongSurfaceOutput moveAlongSurface(AxiomContext ax, MoveAlongSurfaceInput input) {
        ax.log().info("MoveAlongSurface handling", Map.of());
        try {
            NavMeshQuery query = NavMeshUtil.queryFor(input.getNavmesh());
            DefaultQueryFilter filter = NavMeshUtil.defaultFilter();

            float[] startPos = NavMeshUtil.toArr(input.getStart());
            float[] endPos = NavMeshUtil.toArr(input.getEnd());
            if (!NavMeshUtil.isFinite3(startPos) || !NavMeshUtil.isFinite3(endPos)) {
                return MoveAlongSurfaceOutput.newBuilder().setOk(false)
                        .setError("INVALID_INPUT: start/end must be finite").build();
            }

            float[] halfExtents = NavMeshUtil.defaultHalfExtents(input.getNavmesh(), null);
            Result<FindNearestPolyResult> startNear = query.findNearestPoly(startPos, halfExtents, filter);
            if (startNear.failed() || startNear.result.getNearestRef() == 0) {
                return MoveAlongSurfaceOutput.newBuilder().setOk(false)
                        .setError("NO_POLY_NEAR_START: start point is not near any navmesh polygon").build();
            }

            Result<MoveAlongSurfaceResult> result = query.moveAlongSurface(startNear.result.getNearestRef(), startPos,
                    endPos, filter);
            if (result.failed() || result.result == null) {
                return MoveAlongSurfaceOutput.newBuilder().setOk(false)
                        .setError("QUERY_FAILED: move-along-surface did not complete").build();
            }

            MoveAlongSurfaceOutput.Builder out = MoveAlongSurfaceOutput.newBuilder().setOk(true)
                    .setResultPoint(NavMeshUtil.fromArr(result.result.getResultPos()));
            for (long ref : result.result.getVisited()) {
                out.addVisitedPolyRefs(ref);
            }
            return out.build();
        } catch (NavMeshUtil.NavOpException e) {
            return MoveAlongSurfaceOutput.newBuilder().setOk(false).setError(NavMeshUtil.toError(e)).build();
        } catch (Exception e) {
            return MoveAlongSurfaceOutput.newBuilder().setOk(false)
                    .setError("QUERY_FAILED: unexpected error: " + e.getMessage()).build();
        }
    }
}
