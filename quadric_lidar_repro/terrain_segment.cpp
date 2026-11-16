// Geometry-only components for UNORGANIZED terrain clouds. No semantic labels.
#include <cmath>
#include <cstdint>
#include <vector>
extern "C" int terrain_components(const double* xyz, const double* normals,
    const int64_t* neighbors, const int64_t* cells, int n, int k,
    double max_link, double normal_cos, double plane_distance, int32_t* labels) {
    for (int i=0;i<n;++i) labels[i]=-1;
    std::vector<int> queue; queue.reserve(n);
    int label=0;
    for (int seed=0;seed<n;++seed) {
        if(labels[seed]>=0) continue;
        queue.clear();queue.push_back(seed);labels[seed]=label;
        for(size_t pos=0;pos<queue.size();++pos) {
            const int p=queue[pos];
            for(int j=0;j<k;++j) {
                const int q=neighbors[p*k+j];
                if(q==p || labels[q]>=0 || cells[q]!=cells[p]) continue;
                double d2=0, dot=0, dp=0, dq=0;
                for(int axis=0;axis<3;++axis) {
                    const double delta=xyz[3*q+axis]-xyz[3*p+axis];
                    d2+=delta*delta;dot+=normals[3*p+axis]*normals[3*q+axis];
                    dp+=delta*normals[3*p+axis];dq+=delta*normals[3*q+axis];
                }
                if(d2>max_link*max_link || std::fabs(dot)<normal_cos ||
                   std::fabs(dp)>plane_distance || std::fabs(dq)>plane_distance) continue;
                labels[q]=label;queue.push_back(q);
            }
        }
        ++label;
    }
    return label;
}
