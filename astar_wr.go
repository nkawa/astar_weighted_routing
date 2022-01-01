package astar_wr

import (
	"fmt"
	"image/color"
	"math"
)

//Astar planning class
type Astar struct {
	MinX int
	MaxX int
	MinY int
	MaxY int

	XWidth   int
	YWidth   int
	MaxIndex int

	ObjMap [][]bool //if object, it is true

	OpenSet   map[int]*AstarNode
	CloseSet  map[int]*AstarNode
	UpdateObj ModelUpdate
	Current   *AstarNode
}

type Point struct {
	X int
	Y int
}

type ModelUpdate interface {
	UpdateAstar(*Astar, color.RGBA, int) // add waittime
}

func NewAstar(objects [][2]int, objectRadius, resolution float64) *Astar {
	a := &Astar{
		MaxX: 0,
		MaxY: 0,
		MinX: 999999,
		MinY: 999999,
	}

	for _, obj := range objects {
		if obj[0] < a.MinX {
			a.MinX = obj[0]
		}
		if obj[1] < a.MinY {
			a.MinY = obj[1]
		}
		if obj[0] > a.MaxX {
			a.MaxX = obj[0]
		}
		if obj[1] > a.MaxY {
			a.MaxY = obj[1]
		}
	}

	a.XWidth = a.MaxX - a.MinX
	a.YWidth = a.MaxY - a.MinY

	a.ObjMap = make([][]bool, a.XWidth+1)
	for i := 0; i <= a.XWidth; i++ {
		a.ObjMap[i] = make([]bool, a.YWidth+1)
	}

	count := 0
	for _, o := range objects {
		a.ObjMap[o[0]][o[1]] = true
		count += 1
	}
	fmt.Printf("obj count %d", count)
	a.MaxIndex = a.YWidth*a.XWidth - 1
	return a
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
	px := a.MinX + index%a.XWidth
	py := a.MinY + int(index/a.XWidth)
	return px, py
}

func (a Astar) indToIndXY(index int) (int, int) {
	ix := index % a.XWidth
	iy := index / a.XWidth
	return ix, iy
}

func heuristic(n1, n2 *AstarNode) float64 {
	//	w := 1.0
	w := 0.50
	d := w * math.Hypot(float64(n1.Ix)-float64(n2.Ix), float64(n1.Iy)-float64(n2.Iy))
	return d
}

func (a Astar) verifyGrid(index int) bool {
	if index > a.MaxIndex {
		return false
	}
	px, py := a.indToPosXY(index)

	if px < a.MinX {
		return false
	} else if py < a.MinY {
		return false
	} else if px >= a.MaxX {
		return false
	} else if py >= a.MaxY {
		return false
	}

	ix, iy := a.indToIndXY(index)
	if a.ObjMap[ix][iy] {
		return false
	}
	return true
}

func (a Astar) nodeToInd(n *AstarNode) int {
	index := n.Iy*a.XWidth + n.Ix
	return index
}

// Astar planing (sx,sy) is start, (gx,gy) is goal point
func (a *Astar) Plan(sx, sy, gx, gy int) (route [][2]int, err error) {
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
		for key, val := range open_set {
			calCost := val.Cost + heuristic(ngoal, val)
			if calCost < minCost {
				minCost = calCost
				minKey = key
			}
		}
		cId := minKey
		current := open_set[cId]
		if a.UpdateObj != nil {
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
		if a.UpdateObj != nil {
			a.Current = current
			a.UpdateObj.UpdateAstar(a, color.RGBA{0xa0, 0xb0, 0xb0, 0xff}, 0)
		}

		var nId int
		var node *AstarNode
		motion := [8][3]float64{{1.0, 0, 1.0}, {0, 1.0, 1.0}, {-1.0, 0, 1.0}, {0, -1.0, 1.0}, {-1.0, -1.0, math.Sqrt(2)}, {-1.0, 1.0, math.Sqrt(2)}, {1.0, -1.0, math.Sqrt(2)}, {1.0, 1.0, math.Sqrt(2)}}
		for _, v := range motion {
			node = newNode(current.Ix+int(v[0]), current.Iy+int(v[1]), current.Cost+v[2], cId)
			nId = a.nodeToInd(node)

			if !a.verifyGrid(a.nodeToInd(node)) {
				continue
			}

			// in the closed set?
			if _, ok := close_set[nId]; ok {
				continue
			}

			// in the open set?
			if _, ok := open_set[nId]; !ok {
				// add new open set
				if a.UpdateObj != nil {
					a.Current = node
					a.UpdateObj.UpdateAstar(a, color.RGBA{0x00, 0xb0, 0x0b0, 0xff}, 0)
				}
				open_set[nId] = node
			}
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
