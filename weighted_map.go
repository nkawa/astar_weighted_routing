package astar_wr

import (
	"image"
	"image/color"
	"log"
)

// WeightedMap read glay-scale image and generate Image
func WeightedMap(imData image.Image, closeThreth int) ([][2]int, error) {

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
