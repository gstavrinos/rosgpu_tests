__kernel void sumPointCloud(__global float* v){
    unsigned int i = get_global_id(0);
    v[i] = v[i] * v[i] * v[i] * 0.15f;
}
