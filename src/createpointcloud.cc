#include "createpointcloud.h"
#include "dirent.h"

const double f = 1.957945e+03;
const double baseline = 1.177894e-01;
const double cx = 9.167759e+02 - 288, cy = 6.815350e+02 - 176;

MakePointCloud::MakePointCloud() {
}

MakePointCloud::~MakePointCloud() {
}

std::vector<std::string> get_list_files(const std::string &path, const std::string &exten) {
  std::vector<std::string> list;
  list.clear();
  std::string path_f = path + "/" + exten;
#ifdef WIN32  
#ifdef HAVE_WINRT  
  WIN32_FIND_DATAW FindFileData;
#else  
  WIN32_FIND_DATAA FindFileData;
#endif  
  HANDLE hFind;
#ifdef HAVE_WINRT  
  wchar_t wpath[MAX_PATH];
  size_t copied = mbstowcs(wpath, path_f.c_str(), MAX_PATH);
  CV_Assert((copied != MAX_PATH) && (copied != (size_t)-1));
  hFind = FindFirstFileExW(wpath, FindExInfoStandard, &FindFileData, FindExSearchNameMatch, NULL, 0);
#else  
  hFind = FindFirstFileA((LPCSTR)path_f.c_str(), &FindFileData);
#endif  
  if (hFind == INVALID_HANDLE_VALUE) {
    return list;
  } else {
    do {
      if (FindFileData.dwFileAttributes == FILE_ATTRIBUTE_NORMAL ||
        FindFileData.dwFileAttributes == FILE_ATTRIBUTE_ARCHIVE ||
        FindFileData.dwFileAttributes == FILE_ATTRIBUTE_HIDDEN ||
        FindFileData.dwFileAttributes == FILE_ATTRIBUTE_SYSTEM ||
        FindFileData.dwFileAttributes == FILE_ATTRIBUTE_READONLY) {
        char *fname;
#ifdef HAVE_WINRT  
        char fname_tmp[MAX_PATH] = { 0 };
        size_t copied = wcstombs(fname_tmp, FindFileData.cFileName, MAX_PATH);
        CV_Assert((copied != MAX_PATH) && (copied != (size_t)-1));
        fname = fname_tmp;
#else  
        fname = FindFileData.cFileName;
#endif  
        list.push_back(path + "/" + std::string(fname));
      }
    }
#ifdef HAVE_WINRT  
    while (FindNextFileW(hFind, &FindFileData));
#else  
    while (FindNextFileA(hFind, &FindFileData));
#endif  
    FindClose(hFind);
  }
#else
  DIR *dp;
  struct dirent *dirp;
  if ((dp = opendir(path.c_str())) == NULL) {
    return list;
  }
  while ((dirp = readdir(dp)) != NULL) {
    if (dirp->d_type == DT_REG) {
      if (exten.compare("*") == 0)
        list.push_back(static_cast<std::string>(dirp->d_name));
      else
        if (std::string(dirp->d_name).find(exten) != std::string::npos)
          list.push_back(static_cast<std::string>(dirp->d_name));
    }
  }
  closedir(dp);
#endif  
  return list;
}

static const std::map<MakePointCloud::ImageType, std::string> kImageType2Str = {
    std::make_pair(MakePointCloud::ImageType::Left, "left_images"),
    std::make_pair(MakePointCloud::ImageType::Right, "right_images"),
    std::make_pair(MakePointCloud::ImageType::Disparity, "disparity_images"),
    std::make_pair(MakePointCloud::ImageType::LeftColor, "left_colorimages"),
    std::make_pair(MakePointCloud::ImageType::ImageParam, "image_params"),
    std::make_pair(MakePointCloud::ImageType::DisparityDSP, "disparity_images_dsp") };

std::vector<std::string> MakePointCloud::get_collection_image_path(const std::string &collection_path,
                                                                ImageType image_type,
                                                                const std::vector<std::string> &patterns) {
  std::vector<std::string> image_path;
  if (!kImageType2Str.at(image_type).empty()) {
    auto iter = patterns.begin();
    image_path.clear();
    while (image_path.size() == 0 && iter != patterns.end()) {
      auto name_path = kImageType2Str.at(image_type);
      auto full_path = collection_path + "/" + name_path;
      //image_path = get_list_files(collection_path + "/" + kImageType2Str.at(image_type), *iter);
      // Just for mpv disp case
      image_path = get_list_files(collection_path, *iter);
      //ROS_INFO(image_path.back().c_str());
      ++iter;
    }
    std::sort(image_path.begin(), image_path.end());
  }
  return image_path;
}

cv::Mat MakePointCloud::load_image(const std::string &path, const ImageType type) {
  int kHeight = 1024, kWidth = 1344;
  cv::Mat img;
  std::experimental::filesystem::path p(path);
  std::string pattern = p.extension().string();
  std::ifstream ifs;
  switch (type) {
  case ImageType::Left:
  case ImageType::Right:
    if (pattern == ".png") {
      return cv::imread(path, cv::IMREAD_GRAYSCALE);
    } else if (pattern == ".raw") {
      img.create(kHeight, kWidth, CV_8U);
      ifs.open(path, std::ios::binary);
      if (!ifs.is_open()) {
        std::cout << "Cannot open file " << path << std::endl;
        return img;
      }
      ifs.read((char *)img.data, kWidth * kHeight);
      ifs.close();
    }
    break;
  case ImageType::LeftColor:
    if (pattern == ".jpg") {
      return cv::imread(path, cv::IMREAD_COLOR);
    }
    break;
  case ImageType::Disparity:
    if (pattern == ".png") {
      return cv::imread(path, cv::IMREAD_ANYDEPTH);
    } else if (pattern == ".raw") {
      img.create(kHeight, kWidth, CV_16U);
      ifs.open(path, std::ios::binary);
      if (!ifs.is_open()) {
        std::cout << "Cannot open file " << path << std::endl;
        return img;
      }
      ifs.read((char *)img.data, kWidth * kHeight * 2);
      ifs.close();
    }
  case ImageType::DisparityDSP:
    if (pattern == ".png") {
      return cv::imread(path, cv::IMREAD_ANYDEPTH);
    } else if (pattern == ".dat") {
      img.create(kHeight, kWidth, CV_16U);
      ifs.open(path, std::ios::binary);
      if (!ifs.is_open()) {
        std::cout << "Cannot open file " << path << std::endl;
        return img;
      }
      ifs.read((char *)img.data, kWidth * kHeight * 2);
      ifs.close();
    }
  default:
    break;
  }
  return img;
}

void MakePointCloud::ReadImage(const std::string &path, std::vector<std::string> &disparity_images) {
    std::string batch_dir = { R"(/images_001/output_img/mpv_disp/)" };
    std::string collection_path = path + batch_dir;
    ROS_INFO(collection_path.c_str());
    disparity_images = get_collection_image_path(collection_path, MakePointCloud::ImageType::Disparity,{".raw", "*.png", "*.dat" });
    for (auto name : disparity_images) {
      //std::cout<<name<<std::endl;
      //ROS_INFO(name.c_str());
    }
    /*for (auto path : disparity_images) {
        cv::Mat disp = MakePointCloud::load_image(path, MakePointCloud::ImageType::DisparityDSP);
    }*/
    return;
}

/// @brief 
/// @param disparityimg 
/// @param outputclouds : each element in the vector is formed by points deduced from a row of pixel in the disparity image
void MakePointCloud::Disparity2PointCloud(const cv::Mat &disparityimg, 
                                          std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &outputclouds,
                                          pcl::PointCloud<pcl::PointXYZI>::Ptr &entirepointcloud,
                                          std::vector<std::vector<int>> &pixel_index) {
  for (int row=0; row<disparityimg.rows; row++) {
    std::vector<int> row_index;
    pcl::PointCloud<pcl::PointXYZI>::Ptr rowpointcloud(new pcl::PointCloud<pcl::PointXYZI>());    
    for (int col=0; col<disparityimg.cols; col++) {
      ushort disparity = disparityimg.ptr<ushort>(row)[col];
      if (disparity==0) {
        continue;
      }
      double depth = f * baseline / double(disparity) * 32.;
      // TODO 20 time baseline ??
      if (depth==0 || depth>=28.6) {
        continue;
      }
      pcl::PointXYZI point;
      point.x = (col - cx) * depth / f;
      point.y = (row - cy) * depth / f;
      if (point.y < -1.) continue;
      point.z = depth;
      point.intensity = row;
      rowpointcloud->push_back(point);
      entirepointcloud->push_back(point);
      row_index.push_back(col);
    }
    outputclouds.push_back(rowpointcloud);
    pixel_index.push_back(row_index);
  }
  return;
}