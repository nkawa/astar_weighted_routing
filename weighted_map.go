package astar_wr

import (
	"image"
	"image/color"
	"log"
)

const NCOST = 5

// check outer
func (a *Astar) checkOuter(x, y int, cost byte) byte {
	if x > 0 && a.CostMap[x-1][y] > cost {
		return cost + NCOST
	}
	if y > 0 && a.CostMap[x][y-1] > cost {
		return cost + NCOST
	}
	if y < a.MaxY && a.CostMap[x][y+1] > cost {
		return cost + NCOST
	}
	if x < a.MaxX && a.CostMap[x+1][y] > cost {
		return cost + NCOST
	}
	return cost
}

// WeightedMap read glay-scale image and generate Image
func WeightedAstar(objects [][2]int, iteration int) *Astar {
	a := &Astar{
		MaxX: 0,
		MaxY: 0,
	}

	for _, obj := range objects {
		if obj[0] > a.MaxX {
			a.MaxX = obj[0]
		}
		if obj[1] > a.MaxY {
			a.MaxY = obj[1]
		}
	}
	a.Width = a.MaxX + 1
	a.Height = a.MaxY + 1

	a.CostMap = make([][]byte, a.Width)
	for i := 0; i < a.Width; i++ {
		a.CostMap[i] = make([]byte, a.Height)
	}

	bmap := make([][]byte, a.Width)
	for i := 0; i < a.Width; i++ {
		bmap[i] = make([]byte, a.Height)
	}

	count := 0

	for _, o := range objects {
		a.CostMap[o[0]][o[1]] = 0xff
		count += 1
	}
	//	log.Printf("obj count %d", count)

	for iteration > 0 {
		for x := 0; x < a.Width; x++ {
			for y := 0; y < a.Height; y++ {
				bmap[x][y] = a.checkOuter(x, y, a.CostMap[x][y])
			}
		}
		iteration -= 1
		tmpMap := a.CostMap
		a.CostMap = bmap
		bmap = tmpMap
	}

	a.MaxIndex = (a.Width)*(a.Height) - 1
	return a
}

// WeightedMap read glay-scale image and generate Image
func ObjectMap(imData image.Image, closeThreth int) ([][2]int, error) {

	bound := imData.Bounds()
	W := bound.Dx()
	H := bound.Dy()
	log.Printf("file loaded with %dx%d", W, H)

	data := make([][2]int, 0)
	for x := 0; x < W; x++ {
		for y := 0; y < H; y++ {
			oldPix := imData.At(x, y)
			pixel := color.GrayModel.Convert(oldPix)
			pixelU := color.GrayModel.Convert(pixel).(color.Gray).Y
			if int(pixelU) < closeThreth {
				data = append(data, [2]int{x, y})
			}
		}
	}
	return data, nil
}
