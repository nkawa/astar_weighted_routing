package main

import (
	"flag"
	"fmt"
	"image"
	"log"
	"os"

	_ "github.com/jbuchbinder/gopnm" // for reading pnm files

	astar_wr "github.com/nkawa/astar_weighted_routing"
)

var (
	imgFile = flag.String("imgFile", "projection_edit.pgm", "Image file")
)

func findRoute(field *Field, aStar *astar_wr.Astar) {
	log.Printf("Wait for display ready")
	startOk := <-InitChannel
	//	log.Println("findRoute:display ready Wait 3sec", startOk)
	//	time.Sleep(time.Second * 3) // wait 3 seconds
	log.Println("findRoute: do route!")
	if startOk {
		for {
			cp := <-ClickChan
			route, err := aStar.Plan(cp.X0, cp.Y0, cp.X1, cp.Y1) //from point(10,10) to point(120,120)
			if err != nil {
				fmt.Print(err, "\n")
			} else {
				fmt.Print("route length:", len(route), "\n")
			}
			//			SetRoute(field, route)
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

	objects, _ := astar_wr.WeightedMap(imData, 200)
	myField := &Field{
		Objects: objects,
	}
	aStar := astar_wr.NewAstar(objects, 1, 1)
	aStar.UpdateObj = EbitenUpdate{
		EField: myField,
	}
	go findRoute(myField, aStar)
	RunEbiten(myField) // start display

}
