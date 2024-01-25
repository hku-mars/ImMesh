#pragma once
#include <opencv2/opencv.hpp>

class Subdiv2DIndex : public cv::Subdiv2D
{
public :
  Subdiv2DIndex(cv::Rect rectangle);

  //Source code of Subdiv2D: https://github.com/opencv/opencv/blob/master/modules/imgproc/src/subdivision2d.cpp#L762
  //The implementation tweaks getTrianglesList() so that only the indice of the triangle inside the image are returned
  void getTrianglesIndices(std::vector<int> &ind) const;
};

Subdiv2DIndex::Subdiv2DIndex(cv::Rect rectangle) : cv::Subdiv2D{rectangle}
{
}

void Subdiv2DIndex::getTrianglesIndices(std::vector<int> &triangleList) const
{
  triangleList.clear();
  int i, total = (int)(qedges.size() * 4);
  std::vector<bool> edgemask(total, false);
  const bool filterPoints = true;
  cv::Rect2f rect(topLeft.x, topLeft.y, bottomRight.x - topLeft.x, bottomRight.y - topLeft.y);

  for (i = 4; i < total; i += 2)
  {
    if (edgemask[i])
      continue;
    cv::Point2f a, b, c;
    int edge_a = i;
    int indexA = edgeOrg(edge_a, &a) -4;
    if (filterPoints && !rect.contains(a))
      continue;
    int edge_b = getEdge(edge_a, NEXT_AROUND_LEFT);
    int indexB = edgeOrg(edge_b, &b) - 4;
    if (filterPoints && !rect.contains(b))
      continue;
    int edge_c = getEdge(edge_b, NEXT_AROUND_LEFT);
    int indexC = edgeOrg(edge_c, &c) - 4;
    if (filterPoints && !rect.contains(c))
      continue;
    edgemask[edge_a] = true;
    edgemask[edge_b] = true;
    edgemask[edge_c] = true;

    triangleList.push_back(indexA);
    triangleList.push_back(indexB);
    triangleList.push_back(indexC);
  }
}
