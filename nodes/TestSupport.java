package nodes;

import axiom.AxiomContext;

import java.util.List;
import java.util.Map;
import java.util.Optional;

/** Shared no-op AxiomContext factory for node tests — avoids repeating the
 *  same TestContext boilerplate in every *_test.java file. */
final class TestSupport {
    private TestSupport() {}

    static AxiomContext newContext() {
        return new AxiomContext() {
            public Logger log() {
                return new Logger() {
                    public void debug(String m, Map<String, String> a) {}
                    public void info(String m, Map<String, String> a)  {}
                    public void warn(String m, Map<String, String> a)  {}
                    public void error(String m, Map<String, String> a) {}
                };
            }
            public Secrets secrets() { return name -> Optional.empty(); }
            public String executionId() { return "test-execution-id"; }
            public String flowId() { return "test-flow-id"; }
            public String tenantId() { return "test-tenant-id"; }
            public Reflection reflection() {
                return () -> new FlowReflection() {
                    public List<ReflectionNode> nodes() { return List.of(); }
                    public List<ReflectionEdge> edges() { return List.of(); }
                    public List<ReflectionEdge> loopEdges() { return List.of(); }
                    public FlowPosition position() { return new FlowPosition(0, 0, Map.of(), List.of()); }
                    public String graphId() { return ""; }
                };
            }
            public Mutation mutation() {
                return () -> new FlowMutation() {
                    public int addNode(String pkg, String ver, CanvasPosition pos) { return 0; }
                    public void addEdge(int src, int dst, EdgeCondition cond) {}
                };
            }
        };
    }
}
