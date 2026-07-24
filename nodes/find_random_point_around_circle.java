package nodes;

import axiom.AxiomContext;
import gen.Messages.FindRandomPointAroundCircleInput;
import gen.Messages.FindRandomPointAroundCircleOutput;

import org.recast4j.detour.DefaultQueryFilter;
import org.recast4j.detour.FindNearestPolyResult;
import org.recast4j.detour.FindRandomPointResult;
import org.recast4j.detour.NavMeshQuery;
import org.recast4j.detour.PolygonByCircleConstraint;
import org.recast4j.detour.Result;

import java.util.Map;

public class FindRandomPointAroundCircle {

    /**
     * Sample a uniformly-random reachable point within radius of center,
     * constrained to the walkable surface, seeded for determinism.
     */
    public static FindRandomPointAroundCircleOutput findRandomPointAroundCircle(AxiomContext ax, FindRandomPointAroundCircleInput input) {
        ax.log().info("FindRandomPointAroundCircle handling", Map.of());
        try {
            NavMeshQuery query = NavMeshUtil.queryFor(input.getNavmesh());
            DefaultQueryFilter filter = NavMeshUtil.defaultFilter();

            float[] center = NavMeshUtil.toArr(input.getCenter());
            if (!NavMeshUtil.isFinite3(center)) {
                return FindRandomPointAroundCircleOutput.newBuilder().setOk(false)
                        .setError("INVALID_INPUT: center must be finite").build();
            }
            if (!(input.getRadius() >= 0f) || !Float.isFinite(input.getRadius())) {
                return FindRandomPointAroundCircleOutput.newBuilder().setOk(false)
                        .setError("INVALID_INPUT: radius must be >= 0").build();
            }

            float[] halfExtents = NavMeshUtil.defaultHalfExtents(input.getNavmesh(), null);
            Result<FindNearestPolyResult> near = query.findNearestPoly(center, halfExtents, filter);
            if (near.failed() || near.result.getNearestRef() == 0) {
                return FindRandomPointAroundCircleOutput.newBuilder().setOk(true).setFound(false).build();
            }

            NavMeshQuery.FRand frand = new NavMeshQuery.FRand(input.getSeed());
            // Use the strict circle constraint (clips each candidate polygon to
            // its intersection with the search circle) so `radius` is a real
            // bound on the result, not just a search heuristic.
            Result<FindRandomPointResult> result = query.findRandomPointAroundCircle(near.result.getNearestRef(),
                    center, input.getRadius(), filter, frand, PolygonByCircleConstraint.strict());
            if (result.failed() || result.result == null) {
                return FindRandomPointAroundCircleOutput.newBuilder().setOk(true).setFound(false).build();
            }

            return FindRandomPointAroundCircleOutput.newBuilder().setOk(true).setFound(true)
                    .setPolyRef(result.result.getRandomRef())
                    .setPoint(NavMeshUtil.fromArr(result.result.getRandomPt())).build();
        } catch (NavMeshUtil.NavOpException e) {
            return FindRandomPointAroundCircleOutput.newBuilder().setOk(false).setError(NavMeshUtil.toError(e))
                    .build();
        } catch (Exception e) {
            return FindRandomPointAroundCircleOutput.newBuilder().setOk(false)
                    .setError("QUERY_FAILED: unexpected error: " + e.getMessage()).build();
        }
    }
}
