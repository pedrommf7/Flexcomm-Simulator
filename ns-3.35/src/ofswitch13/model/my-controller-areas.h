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

//cada area border router deve ter uma lista de todos os hosts que estao na sua area
// no inicio fazer o algoritmo de djikstra

// calculo do melhor caminho:

// problema de quado uma area se liga a outra area sem ser a area 0

// 1. ver se o host de destino esta na mesma area do host de origem
// 2. se estiver, calcular o caminho mais curto com base no algoritmo de dijkstra
// 3. se nao estiver, calcular o caminho até ao area border router