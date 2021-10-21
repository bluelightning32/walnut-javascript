



## dev env (javascript)

install http-server globally

```
npm i -g http-server
```

if you are changing wasm code, you need to build it and then copy to example folder : `walnut-javascript.js` and `walnut-javascript.wasm`

to start http server with the example run:


## dev - run the example

```
npm run dev
```


## dev - run the example with livereload

If you are only playing with example html and not building wasm, you can put browser and editor side
by side and let livereload refresh browser as soon as you save html file


first install livereload globally

```
npm i -g livereload
```

when developing jsut run

```
npm run dev-live
```


