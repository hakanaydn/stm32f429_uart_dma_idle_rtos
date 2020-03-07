# README #
Bu çalışmada boyu belli olmayan uart dataları DMA ile memory'e kaydediliyor ve interrupt oluşunca semaphore oluşturarak RTOS'daki taskımızı çalıştıyor ve karakterleri uart tx üzerinden yazıyor. Bunun amacı sürekli interrupta sokmak yerine uzunluğu belli olmayan dataları DMA ile kaydedip ve data gelmedi anı idle ile tespit edip semaphoru calistirmaktir.
