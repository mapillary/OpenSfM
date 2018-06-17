# OpenSfM to PMVS dense point cloud reconstruction

Download CMVS: http://www.di.ens.fr/cmvs/

- [Install PMVS/CMVS](#pmvs-installation-hints-ubuntu)
- [PMVS Inputs](#pmvs-inputs)
- [Usage](#usage)

### PMVS Installation Hints (Linux/Ubuntu)
- Installation pointers here: http://www.di.ens.fr/cmvs/documentation.html

- Type `make` in `cmvs/program/main`. Should make three binaries: 
    + `pmvs2`
    + `cmvs`
    + `genOptions`
    
- Most dependencies installed with apt-get: 

    `sudo apt-get install libgsl0-dev libblas-dev libatlas-dev liblapack-dev liblapacke-dev`

- Updated Graclus link: http://www.cs.utexas.edu/users/dml/Software/graclus.html

#### Lapack Errors: 
http://mdda.net/oss-blog/2014-06/building-VisualSFM-on-FC20/

    ERROR : ../base/numeric/mylapack.cc:6:25: fatal error: clapack/f2c.h: No such file or directory

Update `../base/numeric/mylapack.cc`
From: 

    extern "C" {
    #include <clapack/f2c.h>
    #include <clapack/clapack.h>
    };
To: 

    extern "C" {
    //#include <clapack/f2c.h>
    //#include <clapack/clapack.h>
    #include <lapacke.h>
    };
    #define integer int

Update `../base/numeric/mylapack.h`
From:

    static void lls(std::vector<float>& A,
                  std::vector<float>& b,
                  long int width, long int height);

    static void lls(std::vector<double>& A,
                  std::vector<double>& b,
                  long int width, long int height);
To:

    static void lls(std::vector<float>& A,
                  std::vector<float>& b,
                  int width, int height);

    static void lls(std::vector<double>& A,
                  std::vector<double>& b,
                  int width, int height);

#### Accumulate Error:

    ../base/cmvs/bundle.cc: In member function ‘int CMVS::Cbundle::addImagesSub(const std::vector<std::map<int, float> >&)’:
    ../base/cmvs/bundle.cc:1134:52: error: ‘accumulate’ was not declared in this scope
       return accumulate(addnum.begin(), addnum.end(), 0);

Add this to `../base/cmvs/bundle.cc`

    #include <numeric>

#### Stdlib Error: 
    genOption.cc: In function ‘int main(int, char**)’:
    genOption.cc:17:12: error: ‘exit’ was not declared in this scope

Add this to `genOption.cc`

    #include <cstdlib>

### PMVS Inputs 

These are the files that `export_pmvs` generates for PMVS from OpenSfM output. More info: http://www.di.ens.fr/pmvs/documentation.html

- Images: `visualize/%08d.jpg` (radially undistorted)
- Camera Parameters: `txt/%08d.txt`
- Image Co-Visibility file: `vis.dat`
- Options file: `options.txt` (includes mention of `vis.dat`)
- Output directory: `models/`

### Usage

From the root OpenSfM directory, run:

    bin/export_pmvs <path_to_dataset>

There will be an individual pmvs directory for each separate reconstruction. 

To perform the PMVS point cloud reconstruction, run: 

    ./pmvs2 <path_to_dataset>/pmvs/recon0/ pmvs_options.txt 

This will generate files in `<path_to_dataset>/pmvs/recon0/models/` including a `pmvs_options.txt.ply`

**Important:** note that the trailing `/` in `recon0/` is needed.  Otherwise PMVS will fail to find the options file and will give an `Unrecognizable option` warning.
