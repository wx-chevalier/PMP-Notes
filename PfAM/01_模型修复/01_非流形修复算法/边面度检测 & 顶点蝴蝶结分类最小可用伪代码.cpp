#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
using namespace std;

struct Vertex {
  int id;
  // 位置/属性等（UV、法线、权重等）按需补充
  vector<struct Face *> incidentFaces; // 邻接：与该顶点相邻的面
};

struct Edge {
  int id;
  Vertex *a;
  Vertex *b;
  vector<struct Face *> incidentFaces; // 邻接：穿过该边的所有面
};

struct Face {
  int id;
  Vertex *v[3];
  // 可选：缓存三条边
};

// --- 邻接构建：统计每条边的 incident faces，并填充顶点的 incident faces ---
void buildAdjacency(vector<Vertex *> &vertices, vector<Edge *> &edges,
                    vector<Face *> &faces) {
  edges.clear();

  // 用“无向边键”去重：key = (min(va, vb), max(va, vb))
  struct KeyHash {
    size_t operator()(const pair<int, int> &k) const {
      return (static_cast<size_t>(k.first) << 32) ^
             static_cast<size_t>(k.second);
    }
  };
  unordered_map<pair<int, int>, Edge *, KeyHash> edgeMap;

  // 清空顶点 incidentFaces
  for (auto *v : vertices)
    v->incidentFaces.clear();

  for (auto *f : faces) {
    // 顶点邻接
    for (int i = 0; i < 3; ++i) {
      f->v[i]->incidentFaces.push_back(f);
    }
    // 边邻接（三条边）
    for (int i = 0; i < 3; ++i) {
      Vertex *va = f->v[i];
      Vertex *vb = f->v[(i + 1) % 3];
      int aId = va->id, bId = vb->id;
      if (aId > bId)
        swap(aId, bId);
      pair<int, int> key{aId, bId};

      Edge *e = nullptr;
      auto it = edgeMap.find(key);
      if (it == edgeMap.end()) {
        e = new Edge();
        e->id = static_cast<int>(edgeMap.size());
        e->a = va;
        e->b = vb;
        edgeMap[key] = e;
        edges.push_back(e);
      } else {
        e = it->second;
      }
      e->incidentFaces.push_back(f);
    }
  }
}

// --- 边分类（面度）：用于识别非流形边 ---
enum class EdgeType { ManifoldInternal, Boundary, NonManifold, Orphan };

EdgeType classifyEdge(const Edge *e) {
  size_t deg = e->incidentFaces.size();
  if (deg == 2)
    return EdgeType::ManifoldInternal;
  if (deg == 1)
    return EdgeType::Boundary;
  if (deg == 0)
    return EdgeType::Orphan;
  return EdgeType::NonManifold; // deg > 2
}

vector<Edge *> collectNonManifoldEdges(const vector<Edge *> &edges) {
  vector<Edge *> bad;
  for (auto *e : edges) {
    if (classifyEdge(e) == EdgeType::NonManifold)
      bad.push_back(e);
  }
  return bad;
}

// --- 工具：两面是否在顶点 v 处共享同一条“以 v 为端点的边” ---
bool facesShareEdgeAtVertex(Face *f1, Face *f2, Vertex *v) {
  // f 与 v 相邻时，必有两个“相邻顶点”与 v 组成两条边
  unordered_set<int> s1, s2;
  for (int i = 0; i < 3; ++i)
    if (f1->v[i] != v)
      s1.insert(f1->v[i]->id);
  for (int i = 0; i < 3; ++i)
    if (f2->v[i] != v)
      s2.insert(f2->v[i]->id);
  for (int id : s1)
    if (s2.count(id))
      return true; // 共享“v-同一邻点”的边
  return false;
}

// --- 计算顶点的“面扇”连通分量：蝴蝶结顶点会得到多个扇 ---
vector<vector<Face *>> computeFaceFansAtVertex(Vertex *v) {
  const auto &inc = v->incidentFaces;
  int n = static_cast<int>(inc.size());
  vector<vector<int>> adj(n);
  // 面邻接：在 v 处共享同一条边的两面视为相邻
  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      if (facesShareEdgeAtVertex(inc[i], inc[j], v)) {
        adj[i].push_back(j);
        adj[j].push_back(i);
      }
    }
  }
  // BFS 连通分量
  vector<char> vis(n, 0);
  vector<vector<Face *>> fans;
  for (int i = 0; i < n; ++i)
    if (!vis[i]) {
      vector<Face *> comp;
      queue<int> q;
      q.push(i);
      vis[i] = 1;
      while (!q.empty()) {
        int u = q.front();
        q.pop();
        comp.push_back(inc[u]);
        for (int w : adj[u])
          if (!vis[w]) {
            vis[w] = 1;
            q.push(w);
          }
      }
      fans.push_back(comp);
    }
  return fans;
}

// --- 顶点分裂（蝴蝶结）：按面扇复制顶点，并将各扇内的面指向新顶点 ---
Vertex *duplicateVertex(Vertex *v) {
  Vertex *nv = new Vertex();
  nv->id = -1; // 之后应分配全局唯一 id
  // 拷贝位置与所有属性（UV、法线、颜色、骨骼权重等）
  return nv;
}

void replaceVertexInFace(Face *f, Vertex *oldV, Vertex *newV) {
  for (int i = 0; i < 3; ++i) {
    if (f->v[i] == oldV) {
      f->v[i] = newV;
      return;
    }
  }
}

void splitBowTieVertex(Vertex *v, vector<Vertex *> &vertices,
                       vector<Face *> &faces) {
  auto fans = computeFaceFansAtVertex(v);
  if (fans.size() <= 1)
    return; // 非蝴蝶结，无需分裂

  // 保留第一个扇使用原顶点；其它扇复制新顶点并重定向
  for (size_t k = 1; k < fans.size(); ++k) {
    Vertex *nv = duplicateVertex(v);
    // 分配新 id 并注册入顶点表
    nv->id = static_cast<int>(vertices.size());
    vertices.push_back(nv);

    for (Face *f : fans[k]) {
      replaceVertexInFace(f, v, nv);
    }
  }
}

// --- 入口：修复流程（仅拓扑层面） ---
void fixNonManifoldTopology(vector<Vertex *> &vertices, vector<Edge *> &edges,
                            vector<Face *> &faces) {
  // 预邻接
  buildAdjacency(vertices, edges, faces);

  // 顶点蝴蝶结分裂
  for (auto *v : vertices) {
    splitBowTieVertex(v, vertices, faces);
  }

  // 重建邻接，检查边面度（>2 的仍属非流形，之后可做“沿边分裂”）
  buildAdjacency(vertices, edges, faces);
  auto badEdges = collectNonManifoldEdges(edges);
  // 注：对面度>2的边，可采用“按扇对端点复制”的方式进一步分裂；
  // 这里保留为后续步骤，因不同系统的策略（保留哪两面、如何成对）不尽相同。
}