// Range-image connected components; independent implementation, no ROS dependency.
#include <cmath>
#include <cstdint>
#include <vector>
extern "C" int segment(const float* xyz, const float* normals, const int32_t* valid,
                       int h, int w, double angle, double normal_cos, int32_t* labels) {
    const int n=h*w;
    for(int i=0;i<n;i++) labels[i]=-1;
    std::vector<int> queue; queue.reserve(n);
    int label=0;
    for(int seed=0;seed<n;seed++) {
        if(!valid[seed] || labels[seed]>=0) continue;
        queue.clear();queue.push_back(seed);labels[seed]=label;
        for(size_t k=0;k<queue.size();k++) {
            const int p=queue[k], row=p/w,col=p%w;
            const int candidates[4]={row*w+(col+w-1)%w,row*w+(col+1)%w,p-w,p+w};
            for(int side=0;side<4;side++) {
                int q=candidates[side];
                if(q<0 || q>=n || !valid[q] || labels[q]>=0) continue;
                double d1=0,d2=0,dot=0,ndot=0,n1=0,n2=0;
                for(int a=0;a<3;a++) {
                    d1+=xyz[3*p+a]*xyz[3*p+a];d2+=xyz[3*q+a]*xyz[3*q+a];
                    dot+=xyz[3*p+a]*xyz[3*q+a];
                    ndot+=normals[3*p+a]*normals[3*q+a];
                    n1+=normals[3*p+a]*normals[3*p+a];n2+=normals[3*q+a]*normals[3*q+a];
                }
                d1=std::sqrt(d1);d2=std::sqrt(d2);
                double cosine=dot/(d1*d2+1e-15);
                cosine=std::fmax(-1.,std::fmin(1.,cosine));
                const double small=std::fmin(d1,d2),large=std::fmax(d1,d2);
                const double beta=std::atan2(small*std::sqrt(std::fmax(0.,1.-cosine*cosine)),large-small*cosine);
                if(beta<angle) continue;
                if(n1>.5 && n2>.5 && std::fabs(ndot)<normal_cos) continue;
                labels[q]=label;queue.push_back(q);
            }
        }
        label++;
    }
    return label;
}
