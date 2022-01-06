package main

import (
	"flag"
	"fmt"
	"image"
	_ "image/png"
	"log"
	"os"
	"strconv"
	"strings"

	astar_wr "github.com/nkawa/astar_weighted_routing"
)

var (
	imgFile   = flag.String("imgFile", "projection_edit.png", "Image file")
	weight    = flag.Float64("hweight", 0.5, "Weight of Astar heuristic (0->no dist)")
	iteration = flag.Int("iteration", 6, "Iteration for Object range delusion")

//	raduis  = flag.Float64("radius", 2, "Weight object raduis for weight")
//	oweight = flag.Float64("oweight", 1, "Weight of object radius")
)

func main() {
	flag.Usage = func() {
		fmt.Fprintf(flag.CommandLine.Output(), "Usage of %s", os.Args[0])
		fmt.Fprintf(flag.CommandLine.Output(), "  x0,y0,x1,y1   # from-to\n")
		flag.PrintDefaults()
	}
	flag.Parse()
	var X0, Y0, X1, Y1 int

	args := flag.Args()

	if len(args) > 0 { // set x0
		arg := strings.Split(args[0], ",")
		if len(arg) != 4 {
			log.Fatal("Not x0,y0,x1,y1!")
		}
		X0, _ = strconv.Atoi(arg[0])
		Y0, _ = strconv.Atoi(arg[1])
		X1, _ = strconv.Atoi(arg[2])
		Y1, _ = strconv.Atoi(arg[3])
		log.Printf("Routing  %d,%d-%d,%d", X0, Y0, X1, Y1)

	} else {
		flag.Usage()
		os.Exit(1)
	}

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

	route, err := aStar.Plan(X0, Y0, X1, Y1, *weight) //from point(10,10) to point(120,120)

	//	jstr, _ := json.Marshal(route) //, "", "	")
	//	fmt.Print("Output:", jstr, "\n")
	fmt.Printf("[")
	for i := range route {
		p := route[len(route)-i-1] // reverse order
		fmt.Printf("%d,%d,", p[0], p[1])
	}
	fmt.Printf("]\n")

}
