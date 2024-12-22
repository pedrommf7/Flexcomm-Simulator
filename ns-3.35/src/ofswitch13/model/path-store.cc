/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * The GPLv2 License (GPLv2)
 *
 * Copyright (c) 2023 Pedro M. Ferreira
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http: //www.gnu.org/licenses/>.
 *
 * Author: Pedro M. Ferreira <pedro.m.marques@inesctec.pt>
 */

#include "path-store.h"

NS_LOG_COMPONENT_DEFINE ("PathStore");

namespace ns3 {
NS_OBJECT_ENSURE_REGISTERED (PathStore);

PathStore::PathStore () : paths_ ()
{
  NS_LOG_FUNCTION (this);
}

PathStore::~PathStore ()
{
  NS_LOG_FUNCTION (this);
  CleanPaths ();
}

TypeId
PathStore::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::PathStore").SetParent<Object> ().AddConstructor<PathStore> ();
  return tid;
}

std::vector<Ptr<Node>>
PathStore::CheckExists (std::vector<Ptr<Node>> path)
{
  for (auto &p : paths_)
    {
      if (p.first == path)
        return p.first;
    }
  return {};
}

void
PathStore::SortPathsAscending ()
{
  paths_.sort ([] (const auto &lhs, const auto &rhs) { return lhs.second < rhs.second; });
}

void
PathStore::SortPathsDescending ()
{
  paths_.sort ([] (const auto &lhs, const auto &rhs) { return lhs.second > rhs.second; });
}

void
PathStore::AddPath (std::vector<Ptr<Node>> path)
{
  if (CheckExists (path).size () > 0)
    return;
  paths_.emplace_back (std::make_pair (std::move (path), INT_MAX));
  SortPathsAscending ();
}

void
PathStore::AddPath (std::vector<Ptr<Node>> path, int distance)
{
  if (CheckExists (path).size () > 0)
    return;
  paths_.emplace_back (std::make_pair (std::move (path), distance));
  SortPathsAscending ();
}

void
PathStore::RemovePath (std::vector<Ptr<Node>> path)
{
  paths_.remove_if ([path] (const auto &p) { return p.first == path; });
}

std::vector<Ptr<Node>>
PathStore::GetShortestPath ()
{
  SortPathsAscending ();
  return paths_.front ().first;
}

std::vector<Ptr<Node>>
PathStore::GetBiggestPath ()
{
  SortPathsAscending ();
  return paths_.back ().first;
}

std::pair<std::vector<Ptr<Node>>, int>
PathStore::GetShortestPathAndDistance () const
{
  // Get the first path with the shortest distance
  return paths_.front ();
}

std::list<std::pair<std::vector<Ptr<Node>>, int>>
PathStore::GetPaths () const
{
  // Get all the paths and their distances
  return paths_;
}

std::list<std::pair<std::vector<Ptr<Node>>, int>>
PathStore::GetShortestsPathsInRange (int percentage) const
{
  // Get all the paths with a distance in the range of the shortest path distance
  std::list<std::pair<std::vector<Ptr<Node>>, int>> shortestsPaths;
  int shortestDistance = paths_.front ().second;
  int range = shortestDistance * percentage / 100 + shortestDistance;
  for (auto &path : paths_)
    {
      if (path.second <= range)
        shortestsPaths.push_back (path);
      else
        break;
    }
  return shortestsPaths;
}

int
PathStore::GetDistance (std::vector<Ptr<Node>> path)
{
  for (auto &p : paths_)
    {
      if (p.first == path)
        return p.second;
    }

  return -1;
}

void
PathStore::CleanPaths ()
{
  paths_.clear ();
}

void
PathStore::CalculateDistances ()
{
  for (auto &path : paths_)
    {
      int distance = 0;
      for (int i = 0; i < int (path.first.size ()) - 1; i++)
        {
          Ptr<Node> n1 = path.first.at (i);
          Ptr<Node> n2 = path.first.at (i + 1);
          int acc = Topology::GetEdgeWeight (n1, n2);
          distance += acc;
        }

      path.second = distance;
    }

  // sort paths by distance
  PathStore::SortPathsAscending ();
}

int
PathStore::GetNumberOfStoredPaths () const
{
  return paths_.size ();
}

} // namespace ns3
