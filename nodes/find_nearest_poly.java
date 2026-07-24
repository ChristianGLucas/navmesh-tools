package nodes;

import axiom.AxiomContext;
import gen.Messages.FindNearestPolyInput;
import gen.Messages.FindNearestPolyOutput;

import org.recast4j.detour.DefaultQueryFilter;
import org.recast4j.detour.NavMeshQuery;
import org.recast4j.detour.Result;

import java.util.Map;

public class FindNearestPoly {

    /**
     * Snap an arbitrary point to the nearest navmesh polygon within a search
     * box.
     */
    public static FindNearestPolyOutput findNearestPoly(AxiomContext ax, FindNearestPolyInput input) {
        ax.log().info("FindNearestPoly handling", Map.of());
        try {
            NavMeshQuery query = NavMeshUtil.queryFor(input.getNavmesh());
            DefaultQueryFilter filter = NavMeshUtil.defaultFilter();

            float[] point = NavMeshUtil.toArr(input.getPoint());
            if (!NavMeshUtil.isFinite3(point)) {
                return FindNearestPolyOutput.newBuilder().setOk(false)
                        .setError("INVALID_INPUT: point must be finite").build();
            }
            float[] halfExtents = NavMeshUtil.defaultHalfExtents(input.getNavmesh(), input.getSearchHalfExtents());

            Result<org.recast4j.detour.FindNearestPolyResult> result = query.findNearestPoly(point, halfExtents, filter);
            if (result.failed() || result.result == null || result.result.getNearestRef() == 0) {
                return FindNearestPolyOutput.newBuilder().setOk(true).setFound(false).build();
            }

            return FindNearestPolyOutput.newBuilder().setOk(true).setFound(true)
                    .setPolyRef(result.result.getNearestRef())
                    .setNearestPoint(NavMeshUtil.fromArr(result.result.getNearestPos())).build();
        } catch (NavMeshUtil.NavOpException e) {
            return FindNearestPolyOutput.newBuilder().setOk(false).setError(NavMeshUtil.toError(e)).build();
        } catch (Exception e) {
            return FindNearestPolyOutput.newBuilder().setOk(false)
                    .setError("QUERY_FAILED: unexpected error: " + e.getMessage()).build();
        }
    }
}
