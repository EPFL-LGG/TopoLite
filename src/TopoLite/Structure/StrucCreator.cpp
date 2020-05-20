///////////////////////////////////////////////////////////////
//
// StrucCreator.cpp
//
//   Create Topological Interlocking Structure
//
// by Peng SONG ( songpenghit@gmail.com )
// 
// 16/Oct/2018
//
//
///////////////////////////////////////////////////////////////
#include "StrucCreator.h"
#include <tbb/tbb.h>
//**************************************************************************************//
//                                   Initialization
//**************************************************************************************//

template<typename Scalar>
StrucCreator<Scalar>::StrucCreator(shared_ptr<InputVarList> var): TopoObject(var)
{
}

template<typename Scalar>
StrucCreator<Scalar>::~StrucCreator()
{
}

//**************************************************************************************//
//                                  Create Structure
//**************************************************************************************//

template<typename Scalar>
bool StrucCreator<Scalar>::compute(pCrossMesh crossMesh)
{

    float   cutUpper        = getVarList()->template get<float>("cutUpper");
    float   cutLower        = getVarList()->template get<float>("cutLower");

    tbb::tick_count sta = tbb::tick_count::now();
    blocks.clear();
    std::cout << "Number of Parts:\t" << crossMesh->size() << std::endl;
	for (size_t id = 0; id < crossMesh->size(); id++)
	{
		pCross cross = crossMesh->cross(id);
		if(cross){
            pConvexBlock block = make_shared<ConvexBlock<Scalar>>(cross, Vector2(cutUpper, cutLower));
            block->partID = blocks.size();
            blocks.push_back(block);
        }
        else{
            std::cout << "Empty Cross" << std::endl;
        }
	}

	tbb::parallel_for(tbb::blocked_range<size_t>(0, blocks.size()), [&](const tbb::blocked_range<size_t>& r)
	{
		for (size_t id = r.begin(); id != r.end(); ++id)
		{
            if(blocks[id]) blocks[id]->compute();
		}
	});

	std::cout << "Compute Part Geometry:\t" << (tbb::tick_count::now() - sta).seconds() << std::endl;
    std::cout << "Number of Parts:\t" << blocks.size() << std::endl << std::endl;

	return true;
}

template class StrucCreator<double>;
