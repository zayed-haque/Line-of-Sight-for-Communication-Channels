#ifndef INTERMITTENT_MODEL_H
#define INTERMITTENT_MODEL_H

#include <string.h>
#include <iostream>
#include "hash_combine.h"
#include <unordered_map>
#include <buzz/argos/buzz_loop_functions.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/simulator/media/rab_medium.h>
#include <iostream>
#include <fstream> 

#define NUMROBOTS 30
#define INF 9999
#define PERIODOF 3


class CIntermittentModel : public CBuzzLoopFunctions
{

public:
   CIntermittentModel() {}
   virtual ~CIntermittentModel() {}

   virtual void Init(TConfigurationNode &t_tree);

 
   virtual void Reset();

   /**
    * Executes user-defined logic right after a control step is executed.
    */
   virtual void PostStep();

 
    * @return true if the experiment is finished.

   virtual bool IsExperimentFinished();

   /**
    * Executes user-defined destruction logic.
    * This method should undo whatever is done in Init().
    * @see Init()
    */
   virtual void Destroy();

   virtual void BuzzBytecodeUpdated();

private:
   /**
    * Solves the all-pairs shortest paths problem using the
    * Floyd Warshall algorithm
    *
    */
   void FloydWarshall();

   /**
    * resets all the vectors so they can be populated by poststep
   */
   void resetLists();

   void logData();
   

   void findNetworks();
   /**
    * Extracts the path for any two nodes after the Floyd Warshall algorithm is run
    *
    */
   std::vector<std::vector<UInt16>> GetPath(UInt16 i, UInt16 j, UInt16 it = 0);

   int GetNumRobots() const;

private:
   /** A key which stores IDs for nodes A and B */
   typedef std::tuple<ssize_t, ssize_t> ak;

   /** The flow associated to the nodes */
   float* m_vecFlow = new float[NUMROBOTS];

   // int NetworkIn[NUMROBOTS] = {};

   /** Cheap conversion between arbitrary node IDs and integral representation */
   buzzvm_t* m_id_to_key = new buzzvm_t[NUMROBOTS];
   ssize_t *m_id_to_index = new ssize_t[NUMROBOTS];

   /** The adjacency hash of the graph */
   std::unordered_map<ak, UInt16> m_adjacency_hash;

   /** Stores a list of backpointers to the current set of shortest paths for each node pair */
   std::vector<int16_t> m_next[NUMROBOTS][NUMROBOTS];

   /** The output file name */
   std::string m_strOutFile;

   /** The output file stream */

   /** Random number generator */
   CRandom::CRNG *m_pcRNG;

   int stepssince = 0;

   int totalSteps = 0;

   int maxSteps = 7000;

};

#endif
