package nodes;

import axiom.AxiomContext;
import gen.Messages.GetPolyHeightInput;
import gen.Messages.GetPolyHeightOutput;

import org.recast4j.detour.DefaultQueryFilter;
import org.recast4j.detour.FindNearestPolyResult;
import org.recast4j.detour.NavMeshQuery;
import org.recast4j.detour.Result;

import java.util.Map;

public class GetPolyHeight {

    /**
     * Look up the walkable-surface height under a given (x, z) — the
     * polygon nearest the point's x/z is located first, then the surface
     * height within it is interpolated.
     */
    public static GetPolyHeightOutput getPolyHeight(AxiomContext ax, GetPolyHeightInput input) {
        ax.log().info("GetPolyHeight handling", Map.of());
        try {
            NavMeshQuery query = NavMeshUtil.queryFor(input.getNavmesh());
            DefaultQueryFilter filter = NavMeshUtil.defaultFilter();

            float[] point = NavMeshUtil.toArr(input.getPoint());
            if (!NavMeshUtil.isFinite3(point)) {
                return GetPolyHeightOutput.newBuilder().setOk(false)
                        .setError("INVALID_INPUT: point must be finite").build();
            }

            float[] halfExtents = NavMeshUtil.defaultHalfExtents(input.getNavmesh(), null);
            Result<FindNearestPolyResult> near = query.findNearestPoly(point, halfExtents, filter);
            if (near.failed() || near.result.getNearestRef() == 0) {
                return GetPolyHeightOutput.newBuilder().setOk(true).setFound(false).build();
            }

            Result<Float> height = query.getPolyHeight(near.result.getNearestRef(), point);
            if (height.failed() || height.result == null) {
                return GetPolyHeightOutput.newBuilder().setOk(true).setFound(false).build();
            }

            return GetPolyHeightOutput.newBuilder().setOk(true).setFound(true).setHeight(height.result)
                    .setPolyRef(near.result.getNearestRef()).build();
        } catch (NavMeshUtil.NavOpException e) {
            return GetPolyHeightOutput.newBuilder().setOk(false).setError(NavMeshUtil.toError(e)).build();
        } catch (Exception e) {
            return GetPolyHeightOutput.newBuilder().setOk(false)
                    .setError("QUERY_FAILED: unexpected error: " + e.getMessage()).build();
        }
    }
}
