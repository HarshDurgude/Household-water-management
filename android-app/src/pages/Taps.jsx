import TopBar from "../components/TopBar";

export default function Taps() {
  return (
    <div>

      <div className="p-6 space-y-4">
        {["Kitchen", "Bathroom", "Garden"].map((tap) => (
          <div
            key={tap}
            className="flex justify-between items-center bg-card p-4 rounded-xl"
          >
            <span>{tap}</span>
            <span className="text-green-400">Closed</span>
          </div>
        ))}
      </div>
    </div>
  );
}