package nodes;

import axiom.AxiomContext;
import gen.Messages.QueryPolygonsInBoxInput;
import gen.Messages.QueryPolygonsInBoxOutput;

import org.recast4j.detour.DefaultQueryFilter;
import org.recast4j.detour.NavMeshQuery;
import org.recast4j.detour.Status;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class QueryPolygonsInBox {

    /**
     * List every navmesh polygon whose bounds intersect an axis-aligned box.
     */
    public static QueryPolygonsInBoxOutput queryPolygonsInBox(AxiomContext ax, QueryPolygonsInBoxInput input) {
        ax.log().info("QueryPolygonsInBox handling", Map.of());
        try {
            NavMeshQuery query = NavMeshUtil.queryFor(input.getNavmesh());
            DefaultQueryFilter filter = NavMeshUtil.defaultFilter();

            float[] center = NavMeshUtil.toArr(input.getCenter());
            if (!NavMeshUtil.isFinite3(center)) {
                return QueryPolygonsInBoxOutput.newBuilder().setOk(false)
                        .setError("INVALID_INPUT: center must be finite").build();
            }
            float[] halfExtents = NavMeshUtil.defaultHalfExtents(input.getNavmesh(), input.getHalfExtents());

            List<Long> refs = new ArrayList<>();
            Status status = query.queryPolygons(center, halfExtents, filter, (tile, poly, ref) -> refs.add(ref));
            if (status.isFailed()) {
                return QueryPolygonsInBoxOutput.newBuilder().setOk(false)
                        .setError("QUERY_FAILED: box query did not complete").build();
            }

            QueryPolygonsInBoxOutput.Builder out = QueryPolygonsInBoxOutput.newBuilder().setOk(true);
            for (long ref : refs) {
                out.addPolyRefs(ref);
            }
            return out.build();
        } catch (NavMeshUtil.NavOpException e) {
            return QueryPolygonsInBoxOutput.newBuilder().setOk(false).setError(NavMeshUtil.toError(e)).build();
        } catch (Exception e) {
            return QueryPolygonsInBoxOutput.newBuilder().setOk(false)
                    .setError("QUERY_FAILED: unexpected error: " + e.getMessage()).build();
        }
    }
}
