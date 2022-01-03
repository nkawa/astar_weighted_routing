package main

import (
	"flag"
	"fmt"
	"image"
	"log"
	"os"
	"time"

	_ "github.com/jbuchbinder/gopnm" // for reading pnm files

	astar_wr "github.com/nkawa/astar_weighted_routing"
)

var (
	imgFile   = flag.String("imgFile", "projection_edit.pgm", "Image file")
	weight    = flag.Float64("hweight", 0.5, "Weight of Astar heuristic (0->no dist)")
	iteration = flag.Int("iteration", 6, "Iteration for Object range delusion")

//	raduis  = flag.Float64("radius", 2, "Weight object raduis for weight")
//	oweight = flag.Float64("oweight", 1, "Weight of object radius")
)

func SetRoute(field *Field, route [][2]int) {

	for field.pixels == nil {
		log.Printf("Wait for update")
		time.Sleep(time.Millisecond * 300)
	}

	for i := range route {
		p := route[len(route)-i-1] // reverse order
		field.SetPoint(int(p[0]), int(p[1]), 0xff0000)
		time.Sleep(time.Nanosecond * 1)
	}

}

func findRoute(field *Field, aStar *astar_wr.Astar) {
	log.Printf("Wait for display ready")
	startOk := <-InitChannel
	//	log.Println("findRoute:display ready Wait 3sec", startOk)
	//	time.Sleep(time.Second * 3) // wait 3 seconds
	log.Println("findRoute: do route!")
	if startOk {
		for {
			cp := <-ClickChan
			route, err := aStar.Plan(cp.X0, cp.Y0, cp.X1, cp.Y1, *weight) //from point(10,10) to point(120,120)
			if err != nil {
				fmt.Print(err, "\n")
			} else {
				fmt.Print("route length:", len(route), "\n")
			}
			SetRoute(field, route)
		}
	}
}

func main() {
	flag.Parse()

	file, err := os.Open(*imgFile)
	if err != nil {
		log.Fatal(err)
	}
	defer file.Close()

	imData, _, err := image.Decode(file)
	if err != nil {
		log.Fatal(err)
	}

	objects, _ := astar_wr.ObjectMap(imData, 200)
	aStar := astar_wr.WeightedAstar(objects, *iteration)
	myField := &Field{
		Astar:   aStar,
		Objects: objects,
	}
	aStar.UpdateObj = EbitenUpdate{
		EField: myField,
	}
	go findRoute(myField, aStar)
	RunEbiten(myField) // start display

}
