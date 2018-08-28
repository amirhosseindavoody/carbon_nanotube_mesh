#include<complex>
#include<cstdlib>
#include<iostream>
#include<fstream>
#include<map>
#include<string>
#include<omp.h>

#include"cnpy.h"

const int Nx = 128;
const int Ny = 64;
const int Nz = 32;


int main()
{
  typedef long int_t;
  typedef double float_t;

  int n_threads = std::min(omp_get_max_threads(), 32);
  omp_set_num_threads(n_threads);
  std::cout << "maximum number of threads: "<< omp_get_max_threads() << std::endl;

  std::string directory = "/home/amirhossein/research/mesh/cnt_mesh_fiber/";
  std::cout << "input directory: " << directory << std::endl;

  cnpy::NpyArray x = cnpy::npy_load(directory + "single_cnt.pos.x.npy");
  cnpy::NpyArray y = cnpy::npy_load(directory + "single_cnt.pos.y.npy");
  cnpy::NpyArray z = cnpy::npy_load(directory + "single_cnt.pos.z.npy");
  std::cout << "array shape: ";
  int len = 1;
  for (auto&& s: x.shape){
    std::cout << s << ",";
    len *= s;
  }
  std::cout << std::endl;
  std::cout << "total length is: " << len << std::endl;

  float_t *data_x = x.data<float_t>();
  float_t *data_y = y.data<float_t>();
  float_t *data_z = z.data<float_t>();

  int_t d1 = x.shape[0];
  int_t d2 = x.shape[1];

  std::vector<std::vector<float_t>> t(d2, std::vector<float_t>(3, 0));
  std::vector<std::vector<std::vector<float_t>>> pos(d1, t);

  for (int_t i = 0; i < d1; ++i)
  {
    for (int_t j = 0; j < d2; ++j)
    {
      int_t k = i * d2 + j;
      pos[i][j][0] = data_x[k];
      pos[i][j][1] = data_y[k];
      pos[i][j][2] = data_z[k];
    }
  }


  // // print the first row of the matrix for sanity check
  // for (int_t i=0; i< d2; ++i) {
  //   std::cout << pos[0][i][0] << " , " << pos[0][i][1] << " , " << pos[0][i][2] << std::endl;
  // }

  // std::exit(0);

  int_t nh = 5000;
  std::vector<int_t> hist(nh, 0);
  
  float_t radius = 50;
  float_t dr = radius/float_t(nh);


  
  for (int_t i=0; i<d1; ++i) {
    std::cout << i << "\r" << std::flush;
    #pragma omp parallel shared(pos, hist)
    {
      #pragma omp for
      for (int_t j=i+1; j<d1; ++j) {
        float_t min_d = radius + 1;

        for (int_t k=0; k<d2; ++k) {
          for (int_t l=0; l<d2; ++l) {
            float_t d = std::sqrt(std::pow(pos[i][k][0] - pos[j][l][0], 2) + std::pow(pos[i][k][1] - pos[j][l][1], 2) + std::pow(pos[i][k][2] - pos[j][l][2], 2));
            if (d<min_d) {
              min_d = d;
            }
          }
        }

        int_t idx = static_cast<int_t>(min_d/dr);
        if (idx < nh) {
          #pragma omp atomic
          hist[idx] += 1;
        }

      }
    }
  }

  std::ofstream file(directory + "histogram.dat");

  for (auto& h: hist) {
    file << h << " , ";
  }
  file << std::endl;
  file.close();

  return 0;
}
