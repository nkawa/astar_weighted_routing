package main

//go:generate mule hand_edit.png

import (
	"bytes"
	"flag"
	"fmt"
	"image"
	_ "image/png"
	"io"
	"log"
	"math/rand"
	"os"
	"strconv"
	"strings"
	"time"

	astar_wr "github.com/nkawa/astar_weighted_routing"
)

var (
	//	imgFile   = flag.String("imgFile", "projection_edit.png", "Image file")
	//	imgFile   = flag.String("imgFile", "hand_edit.png", "Image file")
	imgFile   = flag.String("imgFile", "", "Image file")
	seed      = flag.Int64("seed", -1, "Random seed for routing")
	rcount    = flag.Int("rcount", 1, "How many routing output")
	weight    = flag.Float64("hweight", 0.5, "Weight of Astar heuristic (0->no dist)")
	iteration = flag.Int("iteration", 6, "Iteration for Object range delusion")
	optimize  = flag.Bool("optimize", false, "Optimize route")

//	raduis  = flag.Float64("radius", 2, "Weight object raduis for weight")
//	oweight = flag.Float64("oweight", 1, "Weight of object radius")
)

// return left/right point from rect
func getPoint(rt [][4]int) (int, int) {
	pt := rt[rand.Intn(len(rt))]
	leftRight := rand.Intn(2) // left or right

	x := pt[leftRight*2]
	dy := pt[3] - pt[1]
	y := pt[1] + rand.Intn(dy+1)

	return x, y
}

func main() {
	flag.Usage = func() {
		fmt.Fprintf(flag.CommandLine.Output(), "Usage of %s", os.Args[0])
		fmt.Fprintf(flag.CommandLine.Output(), "  x0,y0,x1,y1   # from-to\n")
		flag.PrintDefaults()
	}
	flag.Parse()
	var X0, Y0, X1, Y1 int
	var rt [][4]int

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

	} else { // Using random point
		if *imgFile == "" {
			rt = pTokaiRouting()
			if *seed < 0 {
				rand.Seed(time.Now().Unix()) // random seed from current time
			} else {
				rand.Seed(*seed)
			}
			X0, Y0 = getPoint(rt)
			X1, Y1 = getPoint(rt)
		} else {
			flag.Usage()
			os.Exit(1)
		}
	}

	var imgReader io.Reader
	if *imgFile == "" { // need to load from resource
		dt, _ := hand_editResource() // mule (go embedded) generated resource
		imgReader = bytes.NewReader(dt)
	} else {
		file, err := os.Open(*imgFile)
		if err != nil {
			log.Fatal(err)
		}
		defer file.Close()
		imgReader = file
	}

	imData, _, err := image.Decode(imgReader)
	if err != nil {
		log.Fatal(err)
	}

	objects, _ := astar_wr.ObjectMap(imData, 200)
	aStar := astar_wr.WeightedAstar(objects, *iteration)

	//	jstr, _ := json.Marshal(route) //, "", "	")
	//	fmt.Print("Output:", jstr, "\n")
	for *rcount > 0 {
		route, _ := aStar.Plan(X0, Y0, X1, Y1, *weight) //from point(10,10) to point(120,120)

		// optimize?
		if *optimize { //
			route = astar_wr.RouteOptimization(route)
		}

		fmt.Printf("[")
		for i := range route {
			p := route[len(route)-i-1] // reverse order
			fmt.Printf("%d,%d,", p[0], p[1])
		}
		fmt.Printf("]\n")
		X0, Y0 = getPoint(rt)
		X1, Y1 = getPoint(rt)
		*rcount -= 1
	}

}
