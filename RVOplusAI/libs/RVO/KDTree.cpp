//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#include "RVOSimulator.h"
#include "Agent.h"
#include "Obstacle.h"
#include "KDTree.h"

namespace RVO {
  RVOSimulator*  KDTree::_sim = RVOSimulator::Instance();

  KDTree::KDTree(void)
  {
    for (int i = 0; i < (int) _sim->_agents.size(); ++i) {
      _agentIDs.push_back(i);
    }
    _agentTree.resize(2*_sim->_agents.size()-1);

    _obstacleTree = NULL;
  }

  KDTree::~KDTree(void)
  {
    if (_obstacleTree != NULL) {
      deleteObstacleTree(_obstacleTree);
    }
  }

  void KDTree::buildAgentTreeRecursive(int begin, int end, int node) {
    _agentTree[node].begin = begin;
    _agentTree[node].end = end;
    
    _agentTree[node].minX = _agentTree[node].maxX = _sim->_agents[_agentIDs[begin]]->_p.x();
    _agentTree[node].minY = _agentTree[node].maxY = _sim->_agents[_agentIDs[begin]]->_p.y();
    for (int i = begin + 1; i < end; ++i) {
      if (_sim->_agents[_agentIDs[i]]->_p.x() > _agentTree[node].maxX) {
        _agentTree[node].maxX = _sim->_agents[_agentIDs[i]]->_p.x();
      } else if (_sim->_agents[_agentIDs[i]]->_p.x() < _agentTree[node].minX) {
        _agentTree[node].minX = _sim->_agents[_agentIDs[i]]->_p.x();
      }
      if (_sim->_agents[_agentIDs[i]]->_p.y() > _agentTree[node].maxY) {
        _agentTree[node].maxY = _sim->_agents[_agentIDs[i]]->_p.y();
      } else if (_sim->_agents[_agentIDs[i]]->_p.y() < _agentTree[node].minY) {
        _agentTree[node].minY = _sim->_agents[_agentIDs[i]]->_p.y();
      }
    }

    if (end - begin > MAX_LEAF_SIZE) { // no leaf node
      bool vertical = (_agentTree[node].maxX - _agentTree[node].minX > _agentTree[node].maxY - _agentTree[node].minY); // vertical split
      float splitValue = (vertical ? 0.5f * (_agentTree[node].maxX + _agentTree[node].minX) : 0.5f * (_agentTree[node].maxY + _agentTree[node].minY));
      
      int l = begin;
      int r = end - 1;
      while (true) {
        while (l <= r && (vertical ? _sim->_agents[_agentIDs[l]]->_p.x() : _sim->_agents[_agentIDs[l]]->_p.y()) < splitValue) {
          ++l;
        } 
        while (r >= l && (vertical ? _sim->_agents[_agentIDs[r]]->_p.x() : _sim->_agents[_agentIDs[r]]->_p.y()) >= splitValue) {
          --r;
        } 
        if (l > r) {
          break;
        } else {
          std::swap(_agentIDs[l], _agentIDs[r]);
          ++l;
          --r;
        }
      }

      int leftsize = l - begin;

      if (leftsize == 0) {
        ++leftsize;
        ++l;
        ++r;
      }
      
      _agentTree[node].left = node + 1;
      _agentTree[node].right = node + 1 + (2 * leftsize - 1);

      buildAgentTreeRecursive(begin, l, _agentTree[node].left);
      buildAgentTreeRecursive(l, end, _agentTree[node].right);
    }
  }

  void KDTree::queryAgentTreeRecursive(Agent* agent, float& rangeSq, int node) const {
    if (_agentTree[node].end - _agentTree[node].begin <= MAX_LEAF_SIZE) {
      for (int i = _agentTree[node].begin; i < _agentTree[node].end; ++i) {
        agent->insertAgentNeighbor(_agentIDs[i], rangeSq);
      }
    } else {
      float distSqLeft = 0;
      float distSqRight = 0;
      if (agent->_p.x() < _agentTree[_agentTree[node].left].minX) {
        distSqLeft += sqr(_agentTree[_agentTree[node].left].minX - agent->_p.x());
      } else if (agent->_p.x() > _agentTree[_agentTree[node].left].maxX) {
        distSqLeft += sqr(agent->_p.x() - _agentTree[_agentTree[node].left].maxX);
      }
      if (agent->_p.y() < _agentTree[_agentTree[node].left].minY) {
        distSqLeft += sqr(_agentTree[_agentTree[node].left].minY - agent->_p.y());
      } else if (agent->_p.y() > _agentTree[_agentTree[node].left].maxY) {
        distSqLeft += sqr(agent->_p.y() - _agentTree[_agentTree[node].left].maxY);
      }
      if (agent->_p.x() < _agentTree[_agentTree[node].right].minX) {
        distSqRight += sqr(_agentTree[_agentTree[node].right].minX - agent->_p.x());
      } else if (agent->_p.x() > _agentTree[_agentTree[node].right].maxX) {
        distSqRight += sqr(agent->_p.x() - _agentTree[_agentTree[node].right].maxX);
      }
      if (agent->_p.y() < _agentTree[_agentTree[node].right].minY) {
        distSqRight += sqr(_agentTree[_agentTree[node].right].minY - agent->_p.y());
      } else if (agent->_p.y() > _agentTree[_agentTree[node].right].maxY) {
        distSqRight += sqr(agent->_p.y() - _agentTree[_agentTree[node].right].maxY);
      }
      
      if (distSqLeft < distSqRight) {
        if (distSqLeft < rangeSq) {
          queryAgentTreeRecursive(agent, rangeSq, _agentTree[node].left);
          if (distSqRight < rangeSq) {
            queryAgentTreeRecursive(agent, rangeSq, _agentTree[node].right);
          }
        }
      } else {
        if (distSqRight < rangeSq) {
          queryAgentTreeRecursive(agent, rangeSq, _agentTree[node].right);
          if (distSqLeft < rangeSq) {
            queryAgentTreeRecursive(agent, rangeSq, _agentTree[node].left);
          }
        }
      }
      
    }
    
  }

  void KDTree::deleteObstacleTree(ObstacleTreeNode* node) {
    if (node->obstacleID == -1) {
      delete node;
    } else {
      deleteObstacleTree(node->left);
      deleteObstacleTree(node->right);
      delete node;
    }
  }

  KDTree::ObstacleTreeNode* KDTree::buildObstacleTreeRecursive(const std::vector<int>& obstacles) {
    ObstacleTreeNode* node = new ObstacleTreeNode;
    if (obstacles.empty()) {
      node->obstacleID = -1;
      return node;
    } else {
      int optimalSplit = 0;
      int minLeft = (int) obstacles.size();
      int minRight = (int) obstacles.size();
      
      // Compute optimal split node
      for (int i = 0; i < (int)obstacles.size(); ++i) {
        int leftSize = 0;
        int rightSize = 0;
        for (int j = 0; j < (int)obstacles.size(); ++j) {
          if (i != j) {
            float j1_leftof_i = leftOf(_sim->_obstacles[obstacles[i]]->_p1, _sim->_obstacles[obstacles[i]]->_p2, _sim->_obstacles[obstacles[j]]->_p1);
            float j2_leftof_i = leftOf(_sim->_obstacles[obstacles[i]]->_p1, _sim->_obstacles[obstacles[i]]->_p2, _sim->_obstacles[obstacles[j]]->_p2);
            if (j1_leftof_i >= 0 && j2_leftof_i >= 0) {
              ++leftSize;
            } else if (j1_leftof_i <= 0 && j2_leftof_i <= 0) {
              ++rightSize;
            } else {
              ++leftSize;
              ++rightSize;
            }
            if (std::make_pair(std::max(leftSize, rightSize), std::min(leftSize, rightSize)) >= std::make_pair(std::max(minLeft, minRight), std::min(minLeft, minRight))) {
              break;
            }
          }
        }

        if (std::make_pair(std::max(leftSize, rightSize), std::min(leftSize, rightSize)) < std::make_pair(std::max(minLeft, minRight), std::min(minLeft, minRight))) {
          minLeft = leftSize;
          minRight = rightSize;
          optimalSplit = i;
        }
      }

      // Build split node
      std::vector<int> leftObstacles(minLeft);
      std::vector<int> rightObstacles(minRight);
      int leftCounter = 0;
      int rightCounter = 0;
      int i = optimalSplit;
      for (int j = 0; j < (int)obstacles.size(); ++j) {
        if (i != j) {
          float j1_leftof_i = leftOf(_sim->_obstacles[obstacles[i]]->_p1, _sim->_obstacles[obstacles[i]]->_p2, _sim->_obstacles[obstacles[j]]->_p1);
          float j2_leftof_i = leftOf(_sim->_obstacles[obstacles[i]]->_p1, _sim->_obstacles[obstacles[i]]->_p2, _sim->_obstacles[obstacles[j]]->_p2);
          if (j1_leftof_i >= -RVO_EPSILON && j2_leftof_i >= -RVO_EPSILON) {
            leftObstacles[leftCounter++] = obstacles[j];
          } else if (j1_leftof_i <= RVO_EPSILON && j2_leftof_i <= RVO_EPSILON) {
            rightObstacles[rightCounter++] = obstacles[j];
          } else { 
            // split j
            float t = det(_sim->_obstacles[obstacles[i]]->_p2 - _sim->_obstacles[obstacles[i]]->_p1, _sim->_obstacles[obstacles[j]]->_p1 - _sim->_obstacles[obstacles[i]]->_p1) 
                    / det(_sim->_obstacles[obstacles[i]]->_p2 - _sim->_obstacles[obstacles[i]]->_p1, _sim->_obstacles[obstacles[j]]->_p1 - _sim->_obstacles[obstacles[j]]->_p2);
            
            Vector2 splitpoint = _sim->_obstacles[obstacles[j]]->_p1 + t * (_sim->_obstacles[obstacles[j]]->_p2 - _sim->_obstacles[obstacles[j]]->_p1);
            
            Obstacle* new_obstacle = new Obstacle(splitpoint, _sim->_obstacles[obstacles[j]]->_p2); 
            _sim->_obstacles.push_back(new_obstacle);
            int new_obstacle_id = (int) _sim->_obstacles.size() - 1;
            _sim->_obstacles[obstacles[j]]->_p2 = splitpoint;

            if (j1_leftof_i > 0) {
              leftObstacles[leftCounter++] = obstacles[j];
              rightObstacles[rightCounter++] = new_obstacle_id;
            } else {
              rightObstacles[rightCounter++] = obstacles[j];
              leftObstacles[leftCounter++] = new_obstacle_id;
            }
          }
        }
      }

      node->obstacleID = obstacles[optimalSplit];
      node->left = buildObstacleTreeRecursive(leftObstacles);
      node->right = buildObstacleTreeRecursive(rightObstacles);
      return node;
    }
  }
  
  
  void KDTree::buildObstacleTree() {
    if (_obstacleTree != NULL) {
      deleteObstacleTree(_obstacleTree);
    }

    std::vector<int> obstacles(_sim->_obstacles.size());
    for (int i = 0; i < (int)_sim->_obstacles.size(); ++i) {
      obstacles[i] = i;
    }

    _obstacleTree = buildObstacleTreeRecursive(obstacles);
  }

  void KDTree::queryObstacleTreeRecursive(Agent* agent, float& rangeSq, ObstacleTreeNode* node) const {
    if (node->obstacleID == -1) {
      return;
    } else {
      Obstacle* obstacle = _sim->_obstacles[node->obstacleID];
      float agent_leftof_line = leftOf(obstacle->_p1, obstacle->_p2, agent->_p);
      
      queryObstacleTreeRecursive(agent, rangeSq, (agent_leftof_line >= 0 ? node->left : node->right));
        
      float distSqLine = sqr(agent_leftof_line) / absSq(obstacle->_p2 - obstacle->_p1);
      if (distSqLine < rangeSq) { // try obstacle at this node
        agent->insertObstacleNeighbor(node->obstacleID, rangeSq);
        if (distSqLine < rangeSq) { // try other side of line
          queryObstacleTreeRecursive(agent, rangeSq, (agent_leftof_line >= 0 ? node->right : node->left));
        }
      }
    }
  }

  bool KDTree::queryVisibilityRecursive(const Vector2& q1, const Vector2& q2, float radius, ObstacleTreeNode* node) const {
    if (node->obstacleID == -1) {
      return true;
    } else {
      Obstacle* obstacle = _sim->_obstacles[node->obstacleID];
      
      float q1_leftof_i = leftOf(obstacle->_p1, obstacle->_p2, q1);
      float q2_leftof_i = leftOf(obstacle->_p1, obstacle->_p2, q2);

      if (q1_leftof_i >= 0 && q2_leftof_i >= 0) {
        return queryVisibilityRecursive(q1, q2, radius, node->left);
      } else if (q1_leftof_i <= 0 && q2_leftof_i <= 0) {
        return queryVisibilityRecursive(q1, q2, radius, node->right);
      } else {
        float p1_leftof_q = leftOf(q1, q2, obstacle->_p1);
        float p2_leftof_q = leftOf(q1, q2, obstacle->_p2);
        float invLength_q = 1.0f / absSq(q2 - q1);
        
        return (p1_leftof_q * p2_leftof_q >= 0
             && sqr(p1_leftof_q) * invLength_q >= sqr(radius)
             && sqr(p2_leftof_q) * invLength_q >= sqr(radius)
             && queryVisibilityRecursive(q1, q2, radius, node->left) 
             && queryVisibilityRecursive(q1, q2, radius, node->right));
      }
    }
  }

}
