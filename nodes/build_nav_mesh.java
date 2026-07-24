package nodes;

import axiom.AxiomContext;
import gen.Messages.BuildNavMeshInput;
import gen.Messages.BuildNavMeshOutput;
import gen.Messages.NavMesh;
import gen.Messages.Vec3;

import com.google.protobuf.ByteString;

import java.util.Map;

public class BuildNavMesh {

    /**
     * Build a queryable navmesh from raw walkable triangle geometry and an
     * agent config. Deterministic and stateless: the same geometry+agent
     * always yields a byte-identical serialized navmesh.
     */
    public static BuildNavMeshOutput buildNavMesh(AxiomContext ax, BuildNavMeshInput input) {
        ax.log().info("BuildNavMesh handling", Map.of());
        try {
            NavMeshUtil.Built built = NavMeshUtil.buildNavMesh(input.getGeometry(), input.getAgent());
            byte[] data = NavMeshUtil.serialize(built.navMesh);
            NavMesh navmesh = NavMesh.newBuilder()
                    .setData(ByteString.copyFrom(data))
                    .setPolyCount(built.polyCount)
                    .setVertCount(built.vertCount)
                    .setBoundsMin(NavMeshUtil.fromArr(built.bmin))
                    .setBoundsMax(NavMeshUtil.fromArr(built.bmax))
                    .setAgentRadius(input.getAgent().getRadius())
                    .setAgentHeight(input.getAgent().getHeight())
                    .build();
            return BuildNavMeshOutput.newBuilder().setOk(true).setNavmesh(navmesh).build();
        } catch (NavMeshUtil.NavOpException e) {
            return BuildNavMeshOutput.newBuilder().setOk(false).setError(NavMeshUtil.toError(e)).build();
        } catch (Exception e) {
            return BuildNavMeshOutput.newBuilder().setOk(false)
                    .setError("BUILD_FAILED: unexpected error: " + e.getMessage()).build();
        }
    }
}
