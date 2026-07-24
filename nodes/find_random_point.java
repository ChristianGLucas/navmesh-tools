package nodes;

import axiom.AxiomContext;
import gen.Messages.FindRandomPointInput;
import gen.Messages.FindRandomPointOutput;

import org.recast4j.detour.DefaultQueryFilter;
import org.recast4j.detour.FindRandomPointResult;
import org.recast4j.detour.NavMeshQuery;
import org.recast4j.detour.Result;

import java.util.Map;

public class FindRandomPoint {

    /**
     * Sample a uniformly-random point anywhere on the navmesh's walkable
     * surface, seeded for determinism.
     */
    public static FindRandomPointOutput findRandomPoint(AxiomContext ax, FindRandomPointInput input) {
        ax.log().info("FindRandomPoint handling", Map.of());
        try {
            NavMeshQuery query = NavMeshUtil.queryFor(input.getNavmesh());
            DefaultQueryFilter filter = NavMeshUtil.defaultFilter();
            NavMeshQuery.FRand frand = new NavMeshQuery.FRand(input.getSeed());

            Result<FindRandomPointResult> result = query.findRandomPoint(filter, frand);
            if (result.failed() || result.result == null) {
                return FindRandomPointOutput.newBuilder().setOk(false)
                        .setError("QUERY_FAILED: could not sample a random point on this navmesh").build();
            }

            return FindRandomPointOutput.newBuilder().setOk(true).setPolyRef(result.result.getRandomRef())
                    .setPoint(NavMeshUtil.fromArr(result.result.getRandomPt())).build();
        } catch (NavMeshUtil.NavOpException e) {
            return FindRandomPointOutput.newBuilder().setOk(false).setError(NavMeshUtil.toError(e)).build();
        } catch (Exception e) {
            return FindRandomPointOutput.newBuilder().setOk(false)
                    .setError("QUERY_FAILED: unexpected error: " + e.getMessage()).build();
        }
    }
}
