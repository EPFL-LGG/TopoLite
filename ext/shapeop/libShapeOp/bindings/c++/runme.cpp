#include "Solver.h"
#include "Constraint.h"
#include <iostream>

void print_points(const ShapeOp::Matrix34 &p)
{
	for(int i = 0; i < 4; ++ i){
		std::cout << "Point " << i << " : ( ";
		ShapeOp::Vector3 current_pt = p.col(i);
		std::cout << std::scientific << current_pt.transpose();
		std::cout << " )" << std::endl;
	}
}

int main() {
  ShapeOp::Matrix34 p; //column major

  // The << operator reads elements row by row
  p << 0.0, 0.5, 0.5, 0.0,
       0.0, 0.0, 1.0, 1.0,
       0.0, 1.0, 0.0, 1.0;

  std::cout <<  "Input points:" << std::endl;
  print_points(p);

  ShapeOp::Solver s;
  s.setPoints(p);
  ShapeOp::Scalar weight = 1.0;
  //add a plane constraint to all the vertices.
  {
    std::vector<int> id_vector;
    id_vector.push_back(0); id_vector.push_back(1); id_vector.push_back(2); id_vector.push_back(3);
    auto c = std::make_shared<ShapeOp::PlaneConstraint>(id_vector, weight, s.getPoints());
    s.addConstraint(c);
  }
  //add a closeness constraint to the 1st vertex.
  {
    std::vector<int> id_vector;
    id_vector.push_back(0);
    auto c = std::make_shared<ShapeOp::ClosenessConstraint>(id_vector, weight, s.getPoints());
    s.addConstraint(c);
  }
  //add a closeness constraint to the 4th vertex.
  {
    std::vector<int> id_vector;
    id_vector.push_back(3);
    auto c = std::make_shared<ShapeOp::ClosenessConstraint>(id_vector, weight, s.getPoints());
    s.addConstraint(c);
  }
  //add a rigid constraint between 1st and 4th vertex.
  {
    std::vector<int> id_vector;
    id_vector.push_back(0); id_vector.push_back(3);
    auto c = std::make_shared<ShapeOp::SimilarityConstraint>(id_vector, weight, s.getPoints(),false);
    s.addConstraint(c);

  // edit the shapes one of which the rigid constraint brings the involved vertices close to.
    std::vector<ShapeOp::Matrix3X> shapes;
    ShapeOp::Matrix32 s1,s2; //column major

    s1 << 0.0, 0.5,
    	  0.0, 0.0,
		  0.0, 0.0;
    shapes.push_back(s1);

    s2 << 0.0, 1.0,
    	  0.0, 0.0,
		  0.0, 0.0;
    shapes.push_back(s2);

    c->setShapes(shapes);

  }
  s.initialize();
  s.solve(10);
  p = s.getPoints();
  std::cout << "Output points:" << std::endl;
  print_points(p);


  return 0;
}
