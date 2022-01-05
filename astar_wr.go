package astar_wr

import (
	"fmt"
	"image/color"
	"math"
)

//Astar planning class

// in this code, we assume minX = 0, minY = 0
// so, we don't xwid, ywid
type Astar struct {
	MaxX   int
	MaxY   int
	Width  int
	Height int

	MaxIndex int

	CostMap [][]byte //for each object, object COST = 0xff

	OpenSet   map[int]*AstarNode
	CloseSet  map[int]*AstarNode
	UpdateObj ModelUpdate
	Current   *AstarNode
}

var motion = [8][3]float64{
	{1.0, 0, 1.0}, {0, 1.0, 1.0}, {-1.0, 0, 1.0}, {0, -1.0, 1.0},
	{-1.0, -1.0, math.Sqrt(2)}, {-1.0, 1.0, math.Sqrt(2)},
	{1.0, -1.0, math.Sqrt(2)}, {1.0, 1.0, math.Sqrt(2)}}

type Point struct {
	X int
	Y int
}

type ModelUpdate interface {
	UpdateAstar(*Astar, color.RGBA, int) // add waittime
}

type AstarNode struct {
	Index int
	Ix    int
	Iy    int

	Cost      float64
	PrevIndex int

	Obj bool //obstacleならtrue
}

// for astar constructor
func newNode(ix, iy int, cost float64, pind int) *AstarNode {
	n := new(AstarNode)
	n.Ix = ix
	n.Iy = iy
	n.Cost = cost
	n.PrevIndex = pind
	return n
}

func (a Astar) indToPosXY(index int) (int, int) {
	px := index % a.Width
	py := int(index / a.Width)
	return px, py
}

func heuristic(n1, n2 *AstarNode, weight float64) float64 {
	//w := 0.50
	d := weight * math.Hypot(float64(n1.Ix)-float64(n2.Ix), float64(n1.Iy)-float64(n2.Iy))
	return d
}

func (a Astar) verifyGrid(index int) bool {
	if index > a.MaxIndex {
		return false
	}
	px, py := a.indToPosXY(index)
	//	fmt.Printf("verify %d %d : %d\n", px, py, a.CostMap[px][py])
	if px >= a.MaxX {
		return false
	} else if py >= a.MaxY {
		return false
	}

	if a.CostMap[px][py] == 0xff {
		return false
	}
	return true
}

func (a Astar) nodeToInd(n *AstarNode) int {
	index := n.Iy*a.Width + n.Ix
	return index
}

// Astar planing (sx,sy) is start, (gx,gy) is goal point
func (a *Astar) Plan(sx, sy, gx, gy int, weight float64) (route [][2]int, err error) {
	nstart := newNode(sx, sy, 0.0, -1)
	ngoal := newNode(gx, gy, 0.0, -1)

	if !a.verifyGrid(a.nodeToInd(nstart)) {
		err = fmt.Errorf("start point (%d, %d) is not verified", sx, sy)
		return route, err
	}
	if !a.verifyGrid(a.nodeToInd(ngoal)) {
		err = fmt.Errorf("goal point (%d, %d) is not verified", gx, gy)
		return route, err
	}

	open_set := make(map[int]*AstarNode)
	close_set := make(map[int]*AstarNode)
	open_set[a.nodeToInd(nstart)] = nstart // start open position.

	a.OpenSet = open_set
	a.CloseSet = close_set
	if a.UpdateObj != nil {
		a.Current = nstart
		a.UpdateObj.UpdateAstar(a, color.RGBA{0xff, 0, 0, 0xff}, 5)
	}

	for {
		if len(open_set) == 0 {
			err = fmt.Errorf("fail searching point from (%d,%d) to (%d, %d): open set is empty", sx, sy, gx, gy)
			return route, err
		}

		minCost := 1e19
		minKey := -1
		// find minimum cost node
		for key, nextNode := range open_set {
			calCost := nextNode.Cost + heuristic(ngoal, nextNode, weight)
			if calCost < minCost {
				minCost = calCost
				minKey = key
			}
		}
		cId := minKey
		current := open_set[cId] // minimumNode
		if a.UpdateObj != nil {  // update current!
			a.Current = current
			a.UpdateObj.UpdateAstar(a, color.RGBA{0xff, 0, 0, 0xff}, 0)
		}

		if current.Ix == ngoal.Ix && current.Iy == ngoal.Iy {
			//log.Print("find goal")
			ngoal.PrevIndex = current.PrevIndex
			ngoal.Cost = current.Cost
			route = a.finalPath(ngoal, close_set)
			return route, nil
		}

		delete(open_set, cId)

		close_set[cId] = current
		if a.UpdateObj != nil { // display closed data!
			a.Current = current
			a.UpdateObj.UpdateAstar(a, color.RGBA{0xa0, 0xb0, 0xb0, 0xff}, 0)
		}

		var nId int
		var node *AstarNode
		for _, v := range motion {
			nx := current.Ix + int(v[0])
			if nx < 0 || nx > a.MaxX {
				continue
			}
			ny := current.Iy + int(v[1])
			if ny < 0 || ny > a.MaxY {
				continue
			}
			if a.CostMap[nx][ny] == 0xff {
				continue
			}
			// in the closed set?
			nId = nx + ny*a.Width
			if _, ok := close_set[nId]; ok {
				continue
			}
			if _, ok := open_set[nId]; ok {
				continue
			}
			node = newNode(nx, ny, current.Cost+v[2]+float64(a.CostMap[nx][ny]), cId)
			if a.UpdateObj != nil {
				a.Current = node
				a.UpdateObj.UpdateAstar(a, color.RGBA{0x00, 0xb0, 0x0b0, 0xff}, 0)
			}
			// add new openset!
			open_set[nId] = node
		}
	}
}

// 最後に経路の順番にする
func (a Astar) finalPath(ngoal *AstarNode, closeSet map[int]*AstarNode) (route [][2]int) {
	route = append(route, [2]int{ngoal.Ix, ngoal.Iy})

	pind := ngoal.PrevIndex
	for pind != -1 {
		n := closeSet[pind]
		route = append(route, [2]int{n.Ix, n.Iy})
		pind = n.PrevIndex
	}
	return route
}
