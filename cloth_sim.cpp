
#include "raylib.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

static inline double clampd(double x, double a, double b) {
  return (x < a) ? a : (x > b) ? b : x;
}

struct Vec3 {
  double x=0, y=0, z=0;

  Vec3() = default;
  Vec3(double X, double Y, double Z) : x(X), y(Y), z(Z) {}

  Vec3 operator + (const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
  Vec3 operator - (const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
  Vec3 operator * (double s) const { return {x*s, y*s, z*s}; }
  Vec3 operator / (double s) const { return {x/s, y/s, z/s}; }

  Vec3& operator += (const Vec3& o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
  Vec3& operator -= (const Vec3& o) { x-=o.x; y-=o.y; z-=o.z; return *this; }
  Vec3& operator *= (double s) { x*=s; y*=s; z*=s; return *this; }

  double dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
  Vec3 cross(const Vec3& o) const {
    return Vec3{ y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x };
  }
  double norm2() const { return x*x + y*y + z*z; }
  double norm() const { return std::sqrt(norm2()); }

  Vec3 normalized(double eps=1e-12) const {
    double n = norm();
    if (n < eps) return Vec3{0,0,0};
    return *this / n;
  }
};

struct Vec2 {
  double u=0, v=0;
};

struct Tri {
  int a=0,b=0,c=0;
};

// -------------------- XPBD constraints --------------------

struct DistanceConstraint {
  int i=-1, j=-1;
  double rest=0.0;
  double compliance=0.0;
  double lambda=0.0;
};

struct PinConstraint {
  int i=-1;
  Vec3 target;
  double compliance=0.0;
  double lambda=0.0;
};

// -------------------- Colliders --------------------

struct PlaneCollider {
  Vec3 point{0,0,0};
  Vec3 normal{0,1,0}; // unit
  double friction=0.35;
  double bounce=0.0;

  bool project(const Vec3& p, Vec3& out, double& penetration) const {
    double d = (p - point).dot(normal);
    if (d >= 0.0) { out = p; penetration = 0.0; return false; }
    out = p - normal * d;
    penetration = -d;
    return true;
  }
};

struct SphereCollider {
  Vec3 center{0,0,0};
  double radius=1.0;
  double friction=0.25;
  double bounce=0.0;

  bool project(const Vec3& p, Vec3& out, double& penetration, Vec3& n_out) const {
    Vec3 d = p - center;
    double dist = d.norm();
    if (dist >= radius) {
      out = p;
      penetration = 0.0;
      n_out = (dist > 1e-12) ? (d / dist) : Vec3{0,1,0};
      return false;
    }
    Vec3 n = (dist > 1e-12) ? (d / dist) : Vec3{0,1,0};
    out = center + n * radius;
    penetration = radius - dist;
    n_out = n;
    return true;
  }
};

// -------------------- Spatial hashing for self-collision --------------------

struct CellKey {
  int x=0,y=0,z=0;
  bool operator==(const CellKey& o) const { return x==o.x && y==o.y && z==o.z; }
};

struct CellKeyHash {
  std::size_t operator()(const CellKey& k) const noexcept {
    std::size_t h = 1469598103934665603ull;
    auto mix = [&](std::uint64_t v){
      h ^= v + 0x9e3779b97f4a7c15ull + (h<<6) + (h>>2);
    };
    mix((std::uint64_t)(std::uint32_t)k.x);
    mix((std::uint64_t)(std::uint32_t)k.y);
    mix((std::uint64_t)(std::uint32_t)k.z);
    return h;
  }
};

struct SpatialHash {
  double cellSize=0.05;
  std::unordered_map<CellKey, std::vector<int>, CellKeyHash> cells;

  explicit SpatialHash(double h=0.05) : cellSize(h) {}

  CellKey keyFor(const Vec3& p) const {
    return CellKey{
      (int)std::floor(p.x / cellSize),
      (int)std::floor(p.y / cellSize),
      (int)std::floor(p.z / cellSize)
    };
  }

  void build(const std::vector<Vec3>& x) {
    cells.clear();
    cells.reserve(x.size());
    for (int i=0;i<(int)x.size();++i) {
      cells[keyFor(x[i])].push_back(i);
    }
  }

  void gatherNeighbors(const Vec3& p, std::vector<int>& out) const {
    out.clear();
    CellKey k = keyFor(p);
    for (int dx=-1; dx<=1; ++dx)
      for (int dy=-1; dy<=1; ++dy)
        for (int dz=-1; dz<=1; ++dz) {
          CellKey kk{k.x+dx,k.y+dy,k.z+dz};
          auto it = cells.find(kk);
          if (it != cells.end()) {
            const auto& v = it->second;
            out.insert(out.end(), v.begin(), v.end());
          }
        }
  }
};

// -------------------- Cloth mesh --------------------

struct ClothMesh {
  int w=60, h=40;
  double spacing=0.035;
  Vec3 origin{0,1.35,0};

  std::vector<Vec3> x0;
  std::vector<Vec2> uv;
  std::vector<Tri> tris;

  int idx(int i, int j) const { return j*w + i; }

  void buildGrid(int W, int H, double S, const Vec3& O) {
    w=W; h=H; spacing=S; origin=O;
    x0.assign(w*h, Vec3{});
    uv.assign(w*h, Vec2{});
    tris.clear();
    tris.reserve((w-1)*(h-1)*2);

    for (int j=0;j<h;++j) {
      for (int i=0;i<w;++i) {
        double x = (i - (w-1)*0.5) * spacing;
        double z = (j - (h-1)*0.5) * spacing;
        x0[idx(i,j)] = origin + Vec3{x,0.0,z};
        uv[idx(i,j)] = Vec2{ (w>1)? (double)i/(w-1) : 0.0, (h>1)? (double)j/(h-1) : 0.0 };
      }
    }

    for (int j=0;j<h-1;++j) {
      for (int i=0;i<w-1;++i) {
        int a=idx(i,j);
        int b=idx(i+1,j);
        int c=idx(i,j+1);
        int d=idx(i+1,j+1);
        tris.push_back(Tri{a,c,b});
        tris.push_back(Tri{b,c,d});
      }
    }
  }
};

// -------------------- Wind --------------------

struct WindField {
  bool enabled=true;
  Vec3 baseDir{1.0, 0.1, 0.4};
  double strength=8.0;
  double gustStrength=6.0;
  double gustFreq=0.35;
  double turbulence=0.8;

  Vec3 sample(double t, const Vec3& p) const {
    if (!enabled) return Vec3{0,0,0};
    Vec3 d = baseDir.normalized();
    double gust = std::sin(2.0*M_PI*gustFreq*t);
    double g = strength + gustStrength*gust;

    double nx = (std::sin(1.7*p.z + 0.9*t) + std::cos(1.3*p.y + 1.1*t)) * 0.5;
    double ny = (std::sin(1.1*p.x + 1.6*t) + std::cos(1.9*p.z + 0.7*t)) * 0.5;
    double nz = (std::sin(1.5*p.y + 1.2*t) + std::cos(1.2*p.x + 1.4*t)) * 0.5;

    Vec3 noise{nx, ny, nz};
    return d*g + noise*(turbulence*g*0.25);
  }
};

// -------------------- Config --------------------

struct SimConfig {
  double dt = 1.0/60.0;
  int substeps = 2;
  int iters = 14;

  Vec3 gravity{0.0, -9.81, 0.0};
  double linearDamping = 0.015;
  double airDrag = 0.02;

  double compStruct = 2e-7;
  double compShear  = 6e-7;
  double compBend   = 2e-5;
  double compPin    = 0.0;

  double massPerParticle = 0.02;

  bool collisions = true;
  double planeFriction = 0.35;
  double planeBounce = 0.0;
  double sphereFriction = 0.25;
  double sphereBounce = 0.0;

  bool selfCollision = false;
  double selfRadius = 0.018;
  int selfIters = 2;
  double selfCellSize = 0.04;

  bool exportObj = false;
  std::string exportDir = "obj_out";
  int exportFrame = 0;
};

// -------------------- Simulator --------------------

struct ClothSim {
  ClothMesh mesh;
  SimConfig cfg;

  std::vector<Vec3> x, xPrev, v;
  std::vector<double> wInvMass; // inverse mass

  std::vector<DistanceConstraint> distConstraints;
  std::vector<PinConstraint> pinConstraints;

  PlaneCollider plane;
  std::vector<SphereCollider> spheres;

  WindField wind;
  SpatialHash shash;
  std::vector<int> neighBuf;

  double t = 0.0;

  explicit ClothSim(const ClothMesh& m, const SimConfig& c)
    : mesh(m), cfg(c), shash(c.selfCellSize) {
    reset();
    buildConstraints();
    buildDefaultPins();
    setupColliders();
  }

  void setupColliders() {
    plane.point = Vec3{0,0,0};
    plane.normal = Vec3{0,1,0}.normalized();
    plane.friction = cfg.planeFriction;
    plane.bounce = cfg.planeBounce;

    spheres.clear();
    spheres.push_back(SphereCollider{Vec3{0.35, 0.55, 0.05}, 0.22, cfg.sphereFriction, cfg.sphereBounce});
    spheres.push_back(SphereCollider{Vec3{-0.35, 0.40, -0.25}, 0.18, cfg.sphereFriction, cfg.sphereBounce});
  }

  int idx(int i, int j) const { return mesh.idx(i,j); }

  void reset() {
    x = mesh.x0;
    xPrev = mesh.x0;
    v.assign(mesh.x0.size(), Vec3{0,0,0});
    double invm = 1.0 / std::max(1e-12, cfg.massPerParticle);
    wInvMass.assign(mesh.x0.size(), invm);
    t = 0.0;

    // keep pin targets consistent after reset
    for (auto& p : pinConstraints) p.target = x[p.i];
  }

  void buildDefaultPins() {
    pinConstraints.clear();
    auto addPin = [&](int pi) {
      pinConstraints.push_back(PinConstraint{pi, x[pi], cfg.compPin, 0.0});
    };
    addPin(idx(0,0));
    addPin(idx(mesh.w-1,0));
    if (mesh.w > 6) { addPin(idx(2,0)); addPin(idx(mesh.w-3,0)); }
  }

  void addDistance(int i, int j, double compliance) {
    double rest = (mesh.x0[j] - mesh.x0[i]).norm();
    distConstraints.push_back(DistanceConstraint{i, j, rest, compliance, 0.0});
  }

  void buildConstraints() {
    distConstraints.clear();
    distConstraints.reserve(mesh.w*mesh.h*6);

    // structural
    for (int j=0;j<mesh.h;++j) {
      for (int i=0;i<mesh.w;++i) {
        int a = idx(i,j);
        if (i+1 < mesh.w) addDistance(a, idx(i+1,j), cfg.compStruct);
        if (j+1 < mesh.h) addDistance(a, idx(i,j+1), cfg.compStruct);
      }
    }
    // shear
    for (int j=0;j<mesh.h-1;++j) {
      for (int i=0;i<mesh.w-1;++i) {
        int a=idx(i,j);
        int b=idx(i+1,j+1);
        int c=idx(i+1,j);
        int d=idx(i,j+1);
        addDistance(a,b,cfg.compShear);
        addDistance(c,d,cfg.compShear);
      }
    }
    // bend (2-edge)
    for (int j=0;j<mesh.h;++j) {
      for (int i=0;i<mesh.w;++i) {
        int a = idx(i,j);
        if (i+2 < mesh.w) addDistance(a, idx(i+2,j), cfg.compBend);
        if (j+2 < mesh.h) addDistance(a, idx(i,j+2), cfg.compBend);
      }
    }
  }

  void resetLambdas() {
    for (auto& c : distConstraints) c.lambda = 0.0;
    for (auto& p : pinConstraints) p.lambda = 0.0;
  }

  void applyVelocityDamping() {
    double k = clampd(cfg.linearDamping, 0.0, 0.99);
    for (auto& vi : v) vi *= (1.0 - k);
  }

  void applyWindDrag(double dt) {
    if (cfg.airDrag <= 0.0) return;
    for (int i=0;i<(int)x.size();++i) {
      if (wInvMass[i] == 0.0) continue;
      Vec3 wv = wind.sample(t, x[i]);
      v[i] += (wv - v[i]) * (cfg.airDrag * dt);
    }
  }

  void predict(double dt) {
    xPrev = x;
    for (int i=0;i<(int)x.size();++i) {
      if (wInvMass[i] == 0.0) continue;
      v[i] += cfg.gravity * dt;
    }
    applyWindDrag(dt);
    for (int i=0;i<(int)x.size();++i) x[i] += v[i] * dt;
  }

  void solveDistance(DistanceConstraint& c, double dt) {
    double wi = wInvMass[c.i];
    double wj = wInvMass[c.j];
    if (wi==0.0 && wj==0.0) return;

    Vec3 xi = x[c.i];
    Vec3 xj = x[c.j];
    Vec3 d = xj - xi;
    double dist = d.norm();
    if (dist < 1e-12) return;

    double C = dist - c.rest;
    Vec3 n = d / dist;
    Vec3 grad_i = n * (-1.0);
    Vec3 grad_j = n;

    double alpha = c.compliance / (dt*dt);
    double denom = wi * grad_i.norm2() + wj * grad_j.norm2() + alpha;
    if (denom < 1e-20) return;

    double dl = -(C + alpha * c.lambda) / denom;
    c.lambda += dl;

    x[c.i] = xi + grad_i * (wi * dl);
    x[c.j] = xj + grad_j * (wj * dl);
  }

  void solvePin(PinConstraint& p, double dt) {
    double wi = wInvMass[p.i];
    if (wi==0.0) { x[p.i] = p.target; return; }

    Vec3 xi = x[p.i];
    Vec3 d = xi - p.target;
    double dist = d.norm();
    if (dist < 1e-12) return;

    Vec3 grad = d / dist;
    double C = dist;

    double alpha = p.compliance / (dt*dt);
    double denom = wi * grad.norm2() + alpha;
    if (denom < 1e-20) return;

    double dl = -(C + alpha * p.lambda) / denom;
    p.lambda += dl;

    x[p.i] = xi + grad * (wi * dl);
  }

  void solveCollisionsPos() {
    // plane
    for (int i=0;i<(int)x.size();++i) {
      if (wInvMass[i]==0.0) continue;
      Vec3 corr; double pen=0;
      if (plane.project(x[i], corr, pen) && pen > 0.0) x[i] = corr;
    }
    // spheres
    for (const auto& s : spheres) {
      for (int i=0;i<(int)x.size();++i) {
        if (wInvMass[i]==0.0) continue;
        Vec3 corr; double pen=0; Vec3 n;
        if (s.project(x[i], corr, pen, n) && pen > 0.0) x[i] = corr;
      }
    }
  }

  void solveSelfCollision() {
    double r = cfg.selfRadius;
    if (!cfg.selfCollision || r <= 0.0) return;

    const double minDist = 2.0 * r;
    shash.cellSize = cfg.selfCellSize;
    shash.build(x);

    for (int pass=0; pass<std::max(1,cfg.selfIters); ++pass) {
      for (int i=0;i<(int)x.size();++i) {
        if (wInvMass[i]==0.0) continue;
        shash.gatherNeighbors(x[i], neighBuf);
        for (int j : neighBuf) {
          if (j <= i) continue;
          if (wInvMass[j]==0.0) continue;

          Vec3 d = x[j] - x[i];
          double dist = d.norm();
          if (dist < 1e-12 || dist >= minDist) continue;

          Vec3 n = d / dist;
          double wi = wInvMass[i], wj = wInvMass[j];
          double wsum = wi + wj;
          if (wsum <= 0.0) continue;

          double corr = (minDist - dist);
          x[i] -= n * (corr * (wi / wsum));
          x[j] += n * (corr * (wj / wsum));
        }
      }
    }
  }

  void solveConstraints(double dt) {
    resetLambdas();
    for (int it=0; it<cfg.iters; ++it) {
      for (auto& p : pinConstraints) solvePin(p, dt);
      for (auto& c : distConstraints) solveDistance(c, dt);

      if (cfg.collisions) solveCollisionsPos();
      if (cfg.selfCollision) solveSelfCollision();
    }
  }

  void applyCollisionVelocityResponse() {
    // plane
    Vec3 n = plane.normal;
    double mu = clampd(plane.friction, 0.0, 5.0);
    double e  = clampd(plane.bounce, 0.0, 1.0);

    for (int i=0;i<(int)x.size();++i) {
      if (wInvMass[i]==0.0) continue;
      double d = (x[i] - plane.point).dot(n);
      if (d < 1e-6) {
        Vec3 vi = v[i];
        double vn = vi.dot(n);
        Vec3 vt = vi - n * vn;
        if (vn < 0.0) vn = -e * vn;
        vt *= std::max(0.0, 1.0 - mu);
        v[i] = vt + n * vn;
      }
    }

    // spheres
    for (const auto& s : spheres) {
      double muS = clampd(s.friction, 0.0, 5.0);
      double eS  = clampd(s.bounce, 0.0, 1.0);
      for (int i=0;i<(int)x.size();++i) {
        if (wInvMass[i]==0.0) continue;
        Vec3 d = x[i] - s.center;
        double dist = d.norm();
        if (dist < 1e-12) continue;
        if (dist <= s.radius + 1e-6) {
          Vec3 nn = d / dist;
          Vec3 vi = v[i];
          double vn = vi.dot(nn);
          Vec3 vt = vi - nn * vn;
          if (vn < 0.0) vn = -eS * vn;
          vt *= std::max(0.0, 1.0 - muS);
          v[i] = vt + nn * vn;
        }
      }
    }
  }

  void updateVelocities(double dt) {
    double invdt = 1.0 / std::max(1e-12, dt);
    for (int i=0;i<(int)x.size();++i) v[i] = (x[i] - xPrev[i]) * invdt;

    applyVelocityDamping();

    if (cfg.collisions) applyCollisionVelocityResponse();
  }

  void enforcePinsHard() {
    for (auto& p : pinConstraints) {
      x[p.i] = p.target;
      v[p.i] = Vec3{0,0,0};
    }
  }

  void stepFrame() {
    int sub = std::max(1, cfg.substeps);
    double h = cfg.dt / sub;

    for (int s=0; s<sub; ++s) {
      predict(h);
      solveConstraints(h);
      updateVelocities(h);
      t += h;
    }
    enforcePinsHard();

    if (cfg.exportObj) exportOBJ(cfg.exportDir, cfg.exportFrame++);
  }

  void exportOBJ(const std::string& dir, int frameIdx) const {
#if defined(_WIN32)
    std::string mkdirCmd = "mkdir " + dir;
#else
    std::string mkdirCmd = "mkdir -p " + dir;
#endif
    std::system(mkdirCmd.c_str());

    std::ostringstream path;
    path << dir << "/cloth_" << std::setw(5) << std::setfill('0') << frameIdx << ".obj";
    std::ofstream f(path.str());
    if (!f) return;

    f << "# XPBD cloth export\n";
    for (const auto& p : x) {
      f << "v " << std::fixed << std::setprecision(6) << p.x << " " << p.y << " " << p.z << "\n";
    }
    for (const auto& t2 : mesh.uv) {
      f << "vt " << std::fixed << std::setprecision(6) << t2.u << " " << (1.0 - t2.v) << "\n";
    }
    for (const auto& tri : mesh.tris) {
      f << "f "
        << (tri.a+1) << "/" << (tri.a+1) << " "
        << (tri.b+1) << "/" << (tri.b+1) << " "
        << (tri.c+1) << "/" << (tri.c+1) << "\n";
    }
  }
};

// -------------------- Rendering helpers --------------------

static inline Vector3 toRL(const Vec3& v) {
  return Vector3{ (float)v.x, (float)v.y, (float)v.z };
}

static inline float fclamp(float x, float a, float b) {
  return (x < a) ? a : (x > b) ? b : x;
}

static Color shadeFromNormal(const Vec3& n, const Vec3& lightDir) {
  double ndl = std::max(0.0, n.normalized().dot(lightDir.normalized()));
  int base = 55;
  int shade = (int)(base + ndl * 140.0);
  shade = std::max(0, std::min(255, shade));
  return Color{ (unsigned char)shade, (unsigned char)shade, (unsigned char)(shade * 0.95), 255 };
}

// -------------------- Main (realtime) --------------------

int main() {
  // Cloth settings (edit here to change default grid)
  const int W = 70;
  const int H = 45;
  const double spacing = 0.032;

  ClothMesh mesh;
  mesh.buildGrid(W, H, spacing, Vec3{0.0, 1.35, 0.0});

  SimConfig cfg;
  cfg.dt = 1.0/60.0;
  cfg.substeps = 2;
  cfg.iters = 14;
  cfg.selfCollision = false;
  cfg.exportObj = false;
  cfg.exportDir = "obj_out";

  ClothSim sim(mesh, cfg);

  // Window
  const int screenW = 1280;
  const int screenH = 720;
  InitWindow(screenW, screenH, "XPBD Cloth Simulator (raylib)");
  SetTargetFPS(60);

  // Camera (orbital)
  Camera3D cam{};
  cam.position = Vector3{ 2.3f, 1.5f, 2.3f };
  cam.target   = Vector3{ 0.0f, 0.8f, 0.0f };
  cam.up       = Vector3{ 0.0f, 1.0f, 0.0f };
  cam.fovy     = 55.0f;
  cam.projection = CAMERA_PERSPECTIVE;

  bool paused = false;
  bool stepOnce = false;
  bool drawWire = false;
  bool drawPoints = false;

  // Light direction for cheap shading
  Vec3 lightDir{0.4, 1.0, 0.2};

  while (!WindowShouldClose()) {
    // Camera control (raylib built-in orbital controls)
    UpdateCamera(&cam, CAMERA_ORBITAL);

    // Input
    if (IsKeyPressed(KEY_SPACE)) paused = !paused;
    if (IsKeyPressed(KEY_N)) stepOnce = true;
    if (IsKeyPressed(KEY_R)) sim.reset();

    if (IsKeyPressed(KEY_W)) sim.wind.enabled = !sim.wind.enabled;
    if (IsKeyPressed(KEY_S)) sim.cfg.selfCollision = !sim.cfg.selfCollision;
    if (IsKeyPressed(KEY_C)) sim.cfg.collisions = !sim.cfg.collisions;

    if (IsKeyPressed(KEY_O)) sim.cfg.exportObj = !sim.cfg.exportObj;

    if (IsKeyPressed(KEY_T)) drawWire = !drawWire;
    if (IsKeyPressed(KEY_P)) drawPoints = !drawPoints;

    if (IsKeyPressed(KEY_LEFT_BRACKET)) sim.cfg.iters = std::max(1, sim.cfg.iters - 1);
    if (IsKeyPressed(KEY_RIGHT_BRACKET)) sim.cfg.iters = std::min(80, sim.cfg.iters + 1);

    if (IsKeyPressed(KEY_MINUS)) sim.cfg.substeps = std::max(1, sim.cfg.substeps - 1);
    if (IsKeyPressed(KEY_EQUAL)) sim.cfg.substeps = std::min(12, sim.cfg.substeps + 1);

    // Sim step
    if (!paused || stepOnce) {
      sim.stepFrame();
      stepOnce = false;
    }

    // Render
    BeginDrawing();
    ClearBackground(Color{18,18,20,255});

    BeginMode3D(cam);

    // Ground grid
    DrawGrid(30, 0.2f);

    // Colliders
    for (const auto& s : sim.spheres) {
      DrawSphere(toRL(s.center), (float)s.radius, Color{40,40,46,255});
      DrawSphereWires(toRL(s.center), (float)s.radius, 16, 16, Color{90,90,100,255});
    }

    // Cloth triangles (double-sided) with cheap normal shading
    for (const auto& tri : sim.mesh.tris) {
      const Vec3& A = sim.x[tri.a];
      const Vec3& B = sim.x[tri.b];
      const Vec3& C = sim.x[tri.c];

      Vec3 n = (B - A).cross(C - A).normalized();
      Color col = shadeFromNormal(n, lightDir);

      Vector3 a = toRL(A), b = toRL(B), c = toRL(C);

      // Filled (front + back to look like cloth)
      DrawTriangle3D(a, b, c, col);
      DrawTriangle3D(c, b, a, col);

      if (drawWire) {
        DrawLine3D(a, b, Color{28,28,32,255});
        DrawLine3D(b, c, Color{28,28,32,255});
        DrawLine3D(c, a, Color{28,28,32,255});
      }
    }

    // Optional particles
    if (drawPoints) {
      for (int i=0;i<(int)sim.x.size();++i) {
        Color pc = (sim.wInvMass[i] == 0.0) ? Color{200,130,90,255} : Color{160,160,170,255};
        DrawSphere(toRL(sim.x[i]), 0.007f, pc);
      }
    }

    EndMode3D();

    // HUD
    int y = 10;
    DrawText(TextFormat("paused=%s  t=%.2fs", paused ? "true":"false", (float)sim.t), 12, y, 20, Color{220,220,225,255}); y += 22;
    DrawText(TextFormat("grid=%dx%d  particles=%d  tris=%d", sim.mesh.w, sim.mesh.h, (int)sim.x.size(), (int)sim.mesh.tris.size()), 12, y, 20, Color{220,220,225,255}); y += 22;
    DrawText(TextFormat("iters=%d  substeps=%d  dt=%.4f", sim.cfg.iters, sim.cfg.substeps, (float)sim.cfg.dt), 12, y, 20, Color{220,220,225,255}); y += 22;
    DrawText(TextFormat("wind=%s  selfcoll=%s  coll=%s  export=%s",
                        sim.wind.enabled ? "on":"off",
                        sim.cfg.selfCollision ? "on":"off",
                        sim.cfg.collisions ? "on":"off",
                        sim.cfg.exportObj ? "on":"off"),
             12, y, 20, Color{220,220,225,255}); y += 26;

    DrawText("Space pause | N step | R reset | W wind | S selfcoll | C coll | O export | [ ] iters | -/= substeps | T wire | P points",
             12, y, 16, Color{200,200,205,255});

    EndDrawing();
  }

  CloseWindow();
  return 0;
}
